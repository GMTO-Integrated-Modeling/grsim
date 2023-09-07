use std::{env, path::Path, time::Instant};

use crseo::{
    wavefrontsensor::{PhaseSensor, PistonSensor, SegmentCalibration},
    FromBuilder, Gmt, SegmentWiseSensorBuilder, WavefrontSensorBuilder,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients_crseo::OpticalModel;
use gmt_dos_clients_io::optics::WfeRms;

const PYWFS_READOUT: usize = 8;
const PYWFS: usize = 8;
const HDFS: usize = 800;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var("GMT_MODES_PATH", &data_repo);

    let sampling_frequency = 1_000usize; // Hz
    let sim_duration = 1usize;
    let n_sample = HDFS * 10; // sim_duration * sampling_frequency;

    let n_lenslet = 92;
    let n_mode: usize = env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

    // Wavefront phase sensor
    let builder = PhaseSensor::builder()
        .lenslet(n_lenslet, 4)
        .wrapping(760e-9 * 0.5);
    let src_builder = builder.guide_stars(None);

    let m2_modes = "M2_OrthoNormGS36_KarhunenLoeveModes";

    // M2 Karhunen-Loeve segment calibration
    let now = Instant::now();
    let mut slopes_mat = builder.clone().calibrate(
        SegmentCalibration::modes(m2_modes, 0..n_mode, "M2"),
        src_builder.clone(),
    );
    println!(
        "M2 {}modes/segment calibrated in {}s",
        n_mode,
        now.elapsed().as_secs()
    );
    slopes_mat.pseudo_inverse(None).unwrap();

    /// Segment piston calibration
    let piston_builder = PistonSensor::builder().pupil_sampling(builder.pupil_sampling());
    let now = Instant::now();
    let mut piston_mat = piston_builder.calibrate(
        SegmentCalibration::modes(m2_modes, 0..1, "M2"),
        src_builder.clone(),
    );
    println!(
        "M2 {}modes/segment calibrated in {}s",
        1,
        now.elapsed().as_secs()
    );
    piston_mat.pseudo_inverse(None).unwrap();
    let p2m = piston_mat.concat_pinv();
    dbg!(&p2m);

    /// GMT optical model
    let gom = OpticalModel::builder()
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .source(src_builder.clone())
        .atmosphere(
            crseo::Atmosphere::builder().ray_tracing(
                25.5,
                builder.pupil_sampling() as i32,
                0f32,
                sim_duration as f32,
                Some(
                    data_repo
                        .join("ngao_atmophere.bin")
                        .to_str()
                        .unwrap()
                        .to_string(),
                ),
                None,
            ),
        )
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    actorscript! {
        1: gom[WfeRms]$
    }

    Ok(())
}
