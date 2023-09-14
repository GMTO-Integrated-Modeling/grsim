use std::{env, path::Path, time::Instant};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    integrator::Offset,
    leftright::{self, Left as Piston, Right as NoPiston},
    once::Once,
    Integrator, Tick, Timer,
};
use gmt_dos_clients_crseo::{
    crseo::{
        atmosphere,
        wavefrontsensor::{PhaseSensor, PistonSensor, SegmentCalibration},
        Atmosphere, Builder, FromBuilder, Gmt, SegmentWiseSensorBuilder, WavefrontSensorBuilder,
    },
    GuideStar, OpticalModel, ResidualM2modes, ResidualPistonMode, WavefrontSensor,
};
use gmt_dos_clients_io::optics::M2modes;

const PYWFS_READOUT: usize = 8;
const PYWFS: usize = 8;
const HDFS: usize = 800;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let sampling_frequency = 1_000usize; // Hz
    let sim_duration = 1usize;
    let n_sample = sim_duration * sampling_frequency;

    let n_lenslet = 92;
    let n_mode: usize = env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

    // Wavefront phase sensor
    let builder = PhaseSensor::builder()
        .lenslet(n_lenslet, 4)
        .wrapping(760e-9 * 0.5);
    let src_builder = builder.guide_stars(None);

    let m2_modes = "M2_OrthoNormGS36_KarhunenLoeveModes";

    // M2 Karhunen-Loeve segment calibration
    let slopes_mat = {
        let now = Instant::now();
        let mut slopes_mat = builder.clone().calibrate(
            SegmentCalibration::modes(m2_modes, 0..n_mode, "M2"),
            src_builder.clone(),
        );
        eprintln!(
            "M2 {}modes/segment calibrated in {}s",
            n_mode,
            now.elapsed().as_secs()
        );
        slopes_mat.pseudo_inverse(None).unwrap();
        slopes_mat
    };

    // Segment piston calibration
    let piston_builder = PistonSensor::builder().pupil_sampling(builder.pupil_sampling());
    let piston_mat = {
        let now = Instant::now();
        let mut piston_mat = piston_builder.calibrate(
            SegmentCalibration::modes(m2_modes, 0..1, "M2"),
            src_builder.clone(),
        );
        eprintln!(
            "M2 {}modes/segment calibrated in {}s",
            1,
            now.elapsed().as_secs()
        );
        piston_mat.pseudo_inverse(None).unwrap();
        let p2m = piston_mat.concat_pinv();
        dbg!(&p2m);
        piston_mat
    };

    // Atmospher builder
    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(sim_duration as f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );

    // GMT optical model
    let gom = OpticalModel::builder()
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .source(src_builder.clone())
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let sensor = WavefrontSensor::new(builder.build()?, slopes_mat);
    let piston_sensor = WavefrontSensor::new(piston_builder.build()?, piston_mat);

    let timer: Timer = Timer::new(n_sample);

    // let sampler_hdfs_to_pwfs = Pulse::new(1, vec![0f64; 7]);

    // let pwfs_integrator = PwfsIntegrator::single_single(n_mode, 0.5f64);
    let no_piston_ctrl = Integrator::<NoPiston<ResidualM2modes>>::new((n_mode - 1) * 7).gain(0.5);
    let piston_ctrl = Integrator::<Piston<ResidualM2modes>>::new(7).gain(0.5);

    let (split_piston, merge_piston) =
        leftright::split_merge_chunks_at::<ResidualM2modes, M2modes>(n_mode, 1);
    let offset_piston = Once::<ResidualPistonMode>::new();

    type PistonOffset = Offset<ResidualPistonMode>;

    actorscript! {
        #[model(state = completed)]
        1: timer[Tick] -> gom
        1: gom[GuideStar]
            -> sensor("PWFS")[ResidualM2modes]
                -> split_piston("Split Piston\nfrom other modes")[Piston<ResidualM2modes>]
                   -> piston_ctrl("Piston\nControl")[Piston<ResidualM2modes>]!
                      -> merge_piston("Merge Piston\nwith other modes")
        1: split_piston("Split Piston\nfrom other modes")[NoPiston<ResidualM2modes>]
            -> no_piston_ctrl("No Piston Modes\nControl")[NoPiston<ResidualM2modes>]!
                -> merge_piston("Merge Piston\nwith other modes")[M2modes] -> gom
        1: gom[GuideStar] -> piston_sensor("HDFS")
        100: piston_sensor("HDFS")[ResidualPistonMode]! -> offset_piston
        1: offset_piston[PistonOffset] -> piston_ctrl("Piston\nControl")
        1: gom[gmt_dos_clients_io::optics::WfeRms]~
        1: gom[gmt_dos_clients_io::optics::SegmentPiston]~
    }

    Ok(())
}
