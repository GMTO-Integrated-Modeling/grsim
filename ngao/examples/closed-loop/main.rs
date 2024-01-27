use std::{env, fs::File, path::Path};

use crseo::{
    atmosphere,
    wavefrontsensor::{LensletArray, Pyramid},
    Atmosphere, Builder, FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Integrator, Timer};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, GuideStar, OpticalModel, Processor, PyramidCalibrator,
    PyramidMeasurements, ResidualM2modes, WavefrontSensor, WavefrontStats,
};
use gmt_dos_clients_io::optics::{M2modes, SegmentWfeRms, WfeRms};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use interface::Tick;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    // Simulation sampling frequency (1kHz)
    let sampling_frequency = 1000_usize;

    let n_lenslet = 92;
    // let (m2_modes, n_mode) = ("ASM_DDKLs_S7OC04184_675kls", 500);
    let (m2_modes, n_mode) = ("M2_OrthoNormGS36p_KarhunenLoeveModes", 500);

    // Pyramid definition
    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);
    // Optical Model (GMT with 1 guide star on-axis)
    // Atmospher builder
    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(5f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );
    let optical_model = OpticalModel::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, n_mode),
        )
        .source(pym.guide_stars(None))
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    // Pyramid interaction matrix
    let file_name = format!("pymtor_{m2_modes}_{n_mode}.pkl");
    let path = data_repo.join(file_name);
    let mut pymtor: PyramidCalibrator = if !path.exists() {
        let pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, n_mode)
            .n_gpu(8)
            .build()?;
        serde_pickle::to_writer(&mut File::create(&path)?, &pymtor, Default::default())?;
        pymtor
    } else {
        println!("Loading {:?}", &path);
        serde_pickle::from_reader(File::open(&path)?, Default::default())?
    };
    assert_eq!(n_mode, pymtor.n_mode);
    println!("{pymtor}");

    let pym = pym.build()?;
    // Pyramid data processor
    let processor: Processor<_> = Processor::from(&pym);
    // Pyramid wafefront sensor
    let pyramid = WavefrontSensor::<_, 1>::new(pym);
    // Pyramid integral controller (modes>1)
    let pym_ctrl = Integrator::<ResidualM2modes>::new((n_mode - 1) * 7).gain(0.5);
    let stats = WavefrontStats::<1>::default();

    // Scopes definition
    let mut monitor = Monitor::new();
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let metronome: Timer = Timer::new(100);
    // Pyramid reconstructor
    pymtor.h00_estimator()?;
    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    actorscript!(
        #[model(name=closed_loop )]
        #[labels(metronome = "Metronome", calibrator = "Reconstructor", pym_ctrl="High-orders\nintegral controller")]
        1: metronome[Tick] -> optical_model[GuideStar].. -> stats
        1: stats[SegmentWfeRms<-9>].. -> segment_wfe_rms_scope
        1: stats[WfeRms<-9>].. -> wfe_rms_scope
        1: optical_model[GuideStar]
            -> pyramid[DetectorFrame<f32>]
                -> processor[PyramidMeasurements] -> calibrator
        1: calibrator[ResidualM2modes]
            -> pym_ctrl[M2modes]!
                 -> optical_model
    );

    monitor.await?;

    Ok(())
}
