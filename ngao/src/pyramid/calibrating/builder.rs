use std::time::Instant;

use crseo::{
    set_gpu,
    wavefrontsensor::{LensletArray, PyramidBuilder},
    Builder, CrseoError, FromBuilder, Gmt, SourceBuilder, WavefrontSensor, WavefrontSensorBuilder,
};
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use nalgebra as na;
use rayon::prelude::*;

use crate::{Processing, PyramidProcessor};

use super::{CalibratingError, PyramidCalibrator, Segment};

#[derive(Debug, Default)]
pub struct PyramidCalibratorBuilder {
    pub(super) pym: PyramidBuilder,
    pub(super) sids: Vec<u8>,
    pub(super) modes: String,
    pub(super) n_mode: usize,
    pub(super) n_gpu: usize,
    pub(super) n_thread: Option<usize>,
    pub(super) piston_mask_threshold: f32,
}

impl PyramidCalibratorBuilder {
    pub fn n_gpu(mut self, n_gpu: usize) -> Self {
        self.n_gpu = n_gpu;
        self
    }
    pub fn n_thread(mut self, n_thread: usize) -> Self {
        self.n_thread = Some(n_thread);
        self
    }
    pub fn sids(mut self, sids: Vec<u8>) -> Self {
        self.sids = sids;
        self
    }
    pub fn piston_mask_threshold(mut self, piston_mask_threshold: f32) -> Self {
        self.piston_mask_threshold = piston_mask_threshold;
        self
    }
    pub fn build(&self) -> Result<PyramidCalibrator, CalibratingError> {
        let mpb = MultiProgress::new();
        let pbs: Vec<_> = self
            .sids
            .iter()
            .map(|sid| {
                let pb = mpb.add(ProgressBar::new(self.n_mode as u64));
                pb.set_style(
                    ProgressStyle::with_template(
                        "{msg} [{eta_precise}] {bar:30.cyan/blue} {pos:>7}/{len:7}",
                    )
                    .unwrap(),
                );
                pb.set_message(format!("Calibrating segment #{sid}"));
                pb
            })
            .collect();

        let now = Instant::now();
        rayon::ThreadPoolBuilder::new()
            .num_threads(self.n_thread.unwrap_or(self.n_gpu))
            .build_global()
            .unwrap();
        let segments = self
            .sids
            .par_iter()
            .zip(pbs.into_par_iter())
            .map(|(sid, pb)| {
                segment(
                    self.pym.clone(),
                    *sid,
                    self.modes.as_str(),
                    self.n_mode,
                    pb,
                    self.n_gpu,
                )
                .map_err(|e| e.into())
            })
            .collect::<Result<Vec<_>, CalibratingError>>()?;
        println!(
            "Calibration of {} modes in {}s",
            self.n_mode,
            now.elapsed().as_secs()
        );

        // let piston_mask = self.pym.piston_mask(
        //     segments.iter().map(|segment| Some(&segment.mask)),
        //     GmtSegmentation::Outers,
        //     self.pym.guide_stars(None),
        // )?;

        let cum_mask =
            segments
                .iter()
                .skip(1)
                .fold(segments[0].mask.clone_owned(), |mut mask, segment| {
                    mask.iter_mut()
                        .zip(segment.mask.iter())
                        .for_each(|(m1, mi)| *m1 = *m1 || *mi);
                    mask
                });
        let h_filter: Vec<_> = cum_mask.into_iter().cloned().collect();
        let columns: Vec<_> = segments
            .iter()
            .flat_map(|segment| {
                let rows: Vec<_> = segment
                    .calibration
                    .row_iter()
                    .zip(h_filter.iter().cycle())
                    .filter(|(_, f)| **f)
                    .map(|(row, _)| row)
                    .collect();
                na::DMatrix::<f32>::from_rows(&rows)
                    .column_iter()
                    .map(|column| column.clone_owned())
                    .collect::<Vec<_>>()
            })
            .collect();
        let h_matrix = na::DMatrix::from_columns(&columns).remove_column(6 * segments[0].n_mode);
        println!("H: {:?}", h_matrix.shape());

        let n_side_lenslet = self.pym.lenslet_array.n_side_lenslet;
        let mut sx_mask = vec![false; n_side_lenslet * n_side_lenslet];
        let mut sy_mask = vec![false; n_side_lenslet * n_side_lenslet];
        let mut sx_mask_in: Vec<_> = sx_mask
            .iter_mut()
            .zip(&h_filter)
            .filter_map(|(m, &h)| if h { None } else { Some(m) })
            .collect();
        let mut sy_mask_in: Vec<_> = sy_mask
            .iter_mut()
            .zip(&h_filter)
            .filter_map(|(m, &h)| if h { None } else { Some(m) })
            .collect();
        // dbg!((sx_mask_in.len(), sy_mask_in.len()));
        for segment in segments.iter().take(6) {
            let sxy: Vec<_> = segment
                .calibration
                .column(0)
                .iter()
                .zip(h_filter.iter().cycle())
                .filter_map(|(&value, &h)| if h { None } else { Some(value) })
                .collect();
            dbg!(sxy.len());
            let max = sxy
                .iter()
                .map(|x| x.abs())
                .max_by(|x, y| x.partial_cmp(y).unwrap())
                .unwrap();
            dbg!(max);
            let threshold = self.piston_mask_threshold * max;
            let n = sxy.len() / 2;
            sx_mask_in.iter_mut().zip(&sxy[..n]).for_each(|(m, &v)| {
                if v.abs() > threshold {
                    **m = true;
                }
            });
            sy_mask_in.iter_mut().zip(&sxy[n..]).for_each(|(m, &v)| {
                if v.abs() > threshold {
                    **m = true;
                }
            });
        }
        let piston_mask = (sx_mask, sy_mask);

        let p_filter: Vec<_> = piston_mask
            .0
            .iter()
            .chain(piston_mask.1.iter())
            .cloned()
            .collect();
        let columns: Vec<_> = segments
            .iter()
            .flat_map(|segment| {
                let rows: Vec<_> = segment
                    .calibration
                    .row_iter()
                    .zip(p_filter.iter())
                    .filter(|(_, f)| **f)
                    .map(|(row, _)| row)
                    .collect();
                let mat = na::DMatrix::<f32>::from_rows(&rows);
                mat.column_iter()
                    .map(|column| column.clone_owned())
                    .collect::<Vec<_>>()
            })
            .collect();
        let p_matrix = na::DMatrix::from_columns(&columns).remove_column(6 * segments[0].n_mode);
        println!("P: {:?}", p_matrix.shape());

        let mut pymc = PyramidCalibrator {
            n_mode: self.n_mode,
            segments,
            piston_mask,
            h_filter,
            p_filter,
            offset: vec![],
            h_matrix,
            p_matrix,
            estimator: None,
        };

        let mut gmt = Gmt::builder().m2(&self.modes, self.n_mode).build().unwrap();
        let mut src = self.pym.guide_stars(None).build()?;
        let mut pym = self.pym.clone().build()?;
        src.through(&mut gmt).xpupil().through(&mut pym);
        let data = PyramidProcessor::from(&pym).processing();
        pymc.offset = pymc.data(&data);

        Ok(pymc)
    }
}

fn radial_order(i: usize) -> f64 {
    if i == 0 {
        1_f64
    } else {
        (((8. * (i + 1) as f64 - 7.).sqrt() - 1.) * 0.5).floor()
    }
}

fn segment(
    pym: PyramidBuilder,
    sid: u8,
    modes: &str,
    n_mode: usize,
    pb: ProgressBar,
    n_gpu: usize,
) -> Result<Segment, CrseoError> {
    set_gpu(((sid - 1) as usize % n_gpu) as i32);

    let LensletArray { n_side_lenslet, .. } = pym.lenslet_array;

    let mut gmt = Gmt::builder().m2(modes, n_mode).build().unwrap();

    let mask = segment_mask(sid, n_side_lenslet, pym.guide_stars(None));

    let mut src = pym.guide_stars(None).build()?;
    let mut pym = pym.build()?;

    let stroke0 = 25e-9;
    let mut m2_segment_coefs = vec![0f64; n_mode];
    gmt.reset();

    let mut poke_matrix = vec![];

    for j in 0..n_mode {
        pb.inc(1);
        let r = radial_order(j);
        // println!("radial order: {r}");
        let stroke = stroke0 / r.sqrt();

        m2_segment_coefs[j] = stroke;
        gmt.m2_segment_modes(sid, &m2_segment_coefs);
        pym.reset();
        src.through(&mut gmt).xpupil().through(&mut pym);
        let mut push_pull_data = PyramidProcessor::from(&pym).processing();

        m2_segment_coefs[j] = -stroke;
        gmt.m2_segment_modes(sid, &m2_segment_coefs);
        pym.reset();
        src.through(&mut gmt).xpupil().through(&mut pym);
        push_pull_data -= PyramidProcessor::from(&pym).processing();

        let q = (0.5 / stroke) as f32;
        push_pull_data *= q;
        // sx -= _sx;
        // sx *= q;
        poke_matrix.extend(push_pull_data);

        // sy -= _sy;
        // sy *= q;
        // poke_matrix.push(sy.as_slice().to_vec());

        m2_segment_coefs[j] = 0f64;
        gmt.m2_segment_modes(sid, &m2_segment_coefs);
    }
    pb.finish();

    // let poke_matrix: Vec<f32> = poke_matrix.into_iter().flatten().collect();
    let n_slopes = n_side_lenslet.pow(2) * 2;
    let mat = na::DMatrix::<f32>::from_column_slice(n_slopes, n_mode, &poke_matrix);

    Ok(Segment {
        sid,
        n_mode,
        mask,
        calibration: mat,
    })
}

fn segment_mask(
    sid: impl Into<i32>,
    n_side_lenslet: usize,
    src_builder: SourceBuilder,
) -> na::DMatrix<bool> {
    let mut gmt = Gmt::builder().build().unwrap();
    gmt.keep(&[sid.into()]);

    let mut src = src_builder.pupil_sampling(n_side_lenslet).build().unwrap();
    src.through(&mut gmt).xpupil();

    nalgebra::DMatrix::<bool>::from_iterator(
        n_side_lenslet,
        n_side_lenslet,
        src.amplitude().into_iter().rev().map(|x| x > 0f32),
    )
}
