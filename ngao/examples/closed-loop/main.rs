use std::fs::File;
use std::path::Path;
use std::sync::Arc;
use std::{env, mem};

use crseo::wavefrontsensor::{LensletArray, Pyramid};
use crseo::{atmosphere, Atmosphere, Builder, FromBuilder, Gmt, WavefrontSensorBuilder};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{once::Once, Average, Integrator, Timer};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, GuideStar, OpticalModel, Processor, PyramidCalibrator,
    PyramidMeasurements, ResidualM2modes, WavefrontSensor, WavefrontStats,
};
use gmt_dos_clients_io::optics::{
    M2modes, SegmentD7Piston, SegmentDWfe, SegmentPiston, SegmentWfe, SegmentWfeRms, WfeRms,
};
use gmt_dos_clients_scope::server::{Monitor, Scope};
// use interface::units::NM;
use interface::{Data, Read, Tick, UniqueIdentifier, Update, Write, UID};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let n_lenslet = 92;
    let m2_modes = "ASM_DDKLs_S7OC04184_675kls";
    let n_mode: usize = 450; //env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);

    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("calibrating");
    let mut pymtor: PyramidCalibrator = if !path.join("pymtor.pkl").exists() {
        let pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, n_mode)
            .n_thread(8)
            .build()?;
        serde_pickle::to_writer(
            &mut File::create(path.join("pymtor.pkl"))?,
            &pymtor,
            Default::default(),
        )?;
        pymtor
    } else {
        serde_pickle::from_reader(File::open(path.join("pymtor.pkl"))?, Default::default())?
    };

    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(5f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );

    let sampling_frequency = 1_000usize; // Hz

    let optical_model = OpticalModel::builder()
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .source(pym.guide_stars(None))
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let pym = pym.build()?;
    let processor: Processor<_> = Processor::from(&pym);
    let pyramid = WavefrontSensor::<_, 1>::new(pym);
    let pym_ctrl = Integrator::<ResidualM2modes>::new(n_mode * 7).gain(0.5);

    let n_sample = 100;
    let metronome: Timer = Timer::new(n_sample);
    let stats = WavefrontStats::<1>::default();
    // let act_print = Print::<_>::default();
    // let act_print32 = Print::<f32>::default();
    // type WfeRmsNm = NM<WfeRms>;

    let mut monitor = Monitor::new();
    let scope = Scope::<WfeRms>::builder("172.31.26.127:5001", &mut monitor)
        .sampling_period((sampling_frequency as f64).recip())
        .build()?;

    pymtor.h0_estimator()?;
    println!("{pymtor}");
    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    actorscript! {
        #[model(name = highorder, state = completed)]
        1: metronome[Tick]
            -> &optical_model[GuideStar]
                -> &pyramid[DetectorFrame<f32>]
                    -> &processor[PyramidMeasurements]
                        -> calibrator("High-order\nReconstructor")[ResidualM2modes]
                            -> &pym_ctrl[M2modes]! -> &optical_model
        // 1:  &optical_model[GuideStar] -> &stats[SegmentDWfe<-9>] -> &act_print
        1: &optical_model[GuideStar] -> &stats
        1: &stats[WfeRms]$ -> scope
        1: &stats[SegmentWfeRms]$
        1: &stats[SegmentPiston]$
    };

    monitor.join().await?;

    {
        let gom = optical_model.lock().await;
        let mut src = gom.src.lock().unwrap();
        let n = src.pupil_sampling();
        let _: complot::Heatmap = (
            (src.phase().as_slice(), (n, n)),
            Some(complot::Config::new().filename("opd_ho.png")),
        )
            .into();
    }

    let metronome: Timer = Timer::new(n_sample);
    let pym_constrained_recon: nalgebra::DMatrix<f32> = serde_pickle::from_reader(
        File::open(path.join("pym_constrained_recon.pkl"))?,
        Default::default(),
    )?;
    pymtor.set_hp_estimator(pym_constrained_recon);
    // println!("{pymtor}");

    let calibrator: Calibration<PyramidCalibrator> = pymtor.into();

    actorscript! {
      #[model(name = allorders, state = completed, resume = True)]
      1: metronome[Tick]
          -> *optical_model[GuideStar]
              -> *pyramid[DetectorFrame<f32>]
                  -> *processor[PyramidMeasurements]
                      -> &calibrator("All-orders\nReconstructor")[ResidualM2modes]
                          -> *pym_ctrl[M2modes]! -> *optical_model
      // 1:  *optical_model[GuideStar] -> *stats[SegmentDWfe<-9>] -> *act_print
      1: *optical_model[GuideStar] -> *stats
      1: *stats[WfeRms]$
      1: *stats[SegmentWfeRms]$
      1: *stats[SegmentPiston]$
    };

    {
        let gom = optical_model.lock().await;
        let mut src = gom.src.lock().unwrap();
        let n = src.pupil_sampling();
        let _: complot::Heatmap = (
            (src.phase().as_slice(), (n, n)),
            Some(complot::Config::new().filename("opd_all.png")),
        )
            .into();
    }

    let metronome: Timer = Timer::new(n_sample * 5);
    let piston_sensor = Average::<f64, SegmentD7Piston, SegmentAD7Piston>::new(7);
    let merger = Merger::new(n_mode);
    let once = Once::new();
    // let act_print = Print::<(Vec<f64>, Vec<(f64, f64)>)>::default();

    actorscript! {
        #[model(name = pymhdfs, state = completed, resume = True)]
        1: metronome[Tick]
            -> *optical_model[GuideStar]
                -> *pyramid[DetectorFrame<f32>]
                    -> *processor[PyramidMeasurements]
                        -> *calibrator("All-orders\nReconstructor")[ResidualM2modes]
                            -> merger
        1:  *optical_model[GuideStar] -> *stats[SegmentD7Piston] -> piston_sensor
        10: piston_sensor[SegmentAD7Piston] -> once
        1: once[MayBeSegmentAD7Piston] -> merger[ResidualM2modes] -> *pym_ctrl[M2modes]! -> *optical_model
        // 1:  *stats[Wfe<-9>] -> act_print
        1: *optical_model[GuideStar] -> *stats
        1: *stats[WfeRms]$
        1: *stats[SegmentWfeRms]$
        1: *stats[SegmentPiston]$
    };

    {
        let gom = optical_model.lock().await;
        let mut src = gom.src.lock().unwrap();
        dbg!(src.wfe_rms());
        let n = src.pupil_sampling();
        let _: complot::Heatmap = (
            (
                src.phase()
                    .iter()
                    .map(|x| x * 1e9)
                    .collect::<Vec<_>>()
                    .as_slice(),
                (n, n),
            ),
            Some(complot::Config::new().filename("opd.png")),
        )
            .into();
    }

    Ok(())
}

#[derive(Debug, Default)]
pub struct Merger {
    n_mode: usize,
    hdfs: Arc<Option<Vec<f64>>>,
    pym: Vec<f64>,
}

impl Merger {
    pub fn new(n_mode: usize) -> Self {
        Self {
            n_mode,
            ..Default::default()
        }
    }
}

impl Update for Merger {}

impl Read<ResidualM2modes> for Merger {
    fn read(&mut self, data: Data<ResidualM2modes>) {
        self.pym = data.into();
    }
}

impl Write<ResidualM2modes> for Merger {
    fn write(&mut self) -> Option<Data<ResidualM2modes>> {
        if let Some(hdfs) = self.hdfs.as_ref() {
            self.pym
                .chunks_mut(self.n_mode)
                .zip(hdfs.iter())
                .for_each(|(m, dp)| {
                    if (*dp).abs() > 250e-9 {
                        m[0] += dp;
                    }
                });
            let piston = self.pym.chunks(self.n_mode).map(|x| x[0]).sum::<f64>() / 7f64;
            self.pym
                .chunks_mut(self.n_mode)
                .for_each(|x| x[0] -= piston);
        }
        Some(mem::take(&mut self.pym).into())
    }
}

#[derive(UID)]
pub enum SegmentAD7Piston {}
pub enum MayBeSegmentAD7Piston {}
impl UniqueIdentifier for MayBeSegmentAD7Piston {
    type DataType = Option<Vec<f64>>;
}

impl Read<MayBeSegmentAD7Piston> for Merger {
    fn read(&mut self, data: Data<MayBeSegmentAD7Piston>) {
        self.hdfs = data.into_arc();
    }
}
