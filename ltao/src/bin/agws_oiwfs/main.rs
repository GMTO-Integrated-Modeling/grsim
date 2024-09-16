use std::path::Path;

use crseo::{
    gmt::GmtM2,
    imaging::{Detector, LensletArray},
    Atmosphere, Builder, FromBuilder, Gmt, RayTracing, Source,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{print::Print, Integrator, Timer};
use gmt_dos_clients_crseo::{
    calibration::{Calibrate, CalibrationMode},
    sensors::Camera,
    Centroids, DeviceInitialize, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{Dev, Frame, SegmentWfeRms, SensorData, Wavefront, WfeRms},
};
use interface::Tick;
use skyangle::Conversion;

const N_STEP: usize = 10;
const M2_N_MODE: usize = 200;
const OIWFS: usize = 1;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    std::env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("bin")
            .join("agws_oiwfs"),
    );

    let sampling_frequency = 1000f64;
    let atm_builder = Atmosphere::builder().ray_tracing(
        RayTracing::default()
            .field_size(20f64.from_arcmin())
            .duration(30.)
            .filepath("dfs-atmosphere.bin"),
    );

    let gmt_builder = Gmt::builder()
        .m2("Karhunen-Loeve", M2_N_MODE)
        .m1_truss_projection(false);

    // OIWFS TIP-TILT SENSOR
    let oiwfs_tt_om = OpticalModel::<Camera<OIWFS>>::builder()
        .sampling_frequency(sampling_frequency)
        .atmosphere(atm_builder.clone())
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("K"))
        .sensor(Camera::builder().detector(Detector::default().n_px_imagelet(256)))
        .build()?;
    println!("{oiwfs_tt_om}");

    // LTWS ...
    let ltws = Camera::builder()
        .lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(8))
        .lenslet_flux(0.75);
    let mut ltws_centroids = Centroids::try_from(&ltws)?;

    let mut ltws_om_builder = OpticalModel::<Camera<OIWFS>>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder)
        .source(Source::builder().band("V"))
        .sensor(ltws);

    let mut calib_m2_modes = <Centroids as Calibrate<GmtM2>>::calibrate(
        &((&ltws_om_builder).into()),
        CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(2),
    )?;
    calib_m2_modes.pseudoinverse();
    println!("{calib_m2_modes}");

    ltws_om_builder.initialize(&mut ltws_centroids);

    let mut ltws_om = ltws_om_builder.atmosphere(atm_builder).build()?;
    println!("{ltws_om}");
    // ... LTWS

    let integrator = Integrator::new(M2_N_MODE * 7).gain(0.5);

    let timer: Timer = Timer::new(N_STEP);
    let print = Print::default();
    actorscript!(
        #[model(name=agws_oiwfs)]
        #[labels(ltws_om="LTWS",
            integrator="Integrator",
            oiwfs_tt_om="OIWFS",
            print="WFE RMS")]
        1: timer[Tick]
                -> ltws_om[Frame<Dev>]!
                -> ltws_centroids[SensorData]
                    -> calib_m2_modes[M2ASMAsmCommand]
                        -> integrator[M2ASMAsmCommand]
                            -> ltws_om
        1: ltws_om[WfeRms<-9>] -> print
        1: ltws_om[SegmentWfeRms<-9>] -> print
        1: ltws_om[Wavefront]${481*481}
        1: integrator[M2ASMAsmCommand]! -> oiwfs_tt_om[WfeRms<-9>] -> print

    );

    Ok(())
}
