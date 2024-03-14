use std::{env, path::Path};

use crseo::{
    atmosphere,
    wavefrontsensor::{LensletArray, Pyramid},
    Atmosphere, FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Integrator, Timer};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, OpticalModel, Processor, PyramidCalibrator, PyramidMeasurements,
    ResidualM2modes,
};
use gmt_dos_clients_io::optics::{M2modes, SegmentWfeRms, WfeRms};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use interface::Tick;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);

    // Simulation sampling frequency (1kHz)
    let sampling_frequency = 1000_usize;

    let n_lenslet = 92;
    // let (m2_modes, n_mode) = ("ASM_DDKLs_S7OC04184_675kls", 500);
    let (m2_modes, n_mode) = ("M2_OrthoNormGS36p90_KarhunenLoeveModes", 500);

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
    let optical_model = OpticalModel::<Pyramid>::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, n_mode),
        )
        .source(pym.guide_stars(None))
        .sensor(pym.clone())
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    // Pyramid interaction matrix
    let calibrator: Calibration<PyramidCalibrator> = {
        let filename = format!("{0}/pym-{m2_modes}-{n_mode}.bin", env!("GMT_MODES_PATH"));
        if let Ok(pymtor) = PyramidCalibrator::try_from(filename.as_str()) {
            pymtor
        } else {
            let mut pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, n_mode)
                .n_gpu(7)
                .build()?;
            pymtor.h00_estimator()?;
            pymtor.save(filename)?
        }
        .into()
    };
    assert_eq!(n_mode, calibrator.n_mode);
    // println!("{pymtor}");

    // Pyramid data processor
    let processor: Processor<_> = Processor::try_from(&pym)?;
    // Pyramid integral controller (modes>1)
    let pym_ctrl = Integrator::<ResidualM2modes>::new(n_mode * 7).gain(0.5);

    // Scopes definition
    let mut monitor = Monitor::new();
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let metronome: Timer = Timer::new(100);

    actorscript!(
    #[model(name=closed_loop )]
    #[labels(metronome = "Metronome", calibrator = "Reconstructor", pym_ctrl="High-orders\nintegral controller")]
        1: metronome[Tick]
           -> optical_model[DetectorFrame]
                -> processor[PyramidMeasurements]
                    -> calibrator[ResidualM2modes]
                        -> pym_ctrl[M2modes]!
                             -> optical_model
        1: optical_model[SegmentWfeRms<-9>].. -> segment_wfe_rms_scope
        1: optical_model[WfeRms<-9>].. -> wfe_rms_scope
    );

    monitor.await?;

    Ok(())
}
