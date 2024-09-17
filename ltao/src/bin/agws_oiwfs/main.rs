use std::path::Path;

use crseo::{
    gmt::GmtM2,
    imaging::{Detector, LensletArray},
    Atmosphere, Builder, FromBuilder, Gmt, RayTracing, Source,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    operator::{Left, Operator, Right},
    print::Print,
    Integrator, Timer,
};
use gmt_dos_clients_crseo::{
    calibration::{Calibrate, CalibrationMode, Reconstructor},
    centroiding::{Full, ZeroMean},
    sensors::Camera,
    Centroids, DeviceInitialize, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{Dev, Frame, SegmentWfeRms, SensorData, Wavefront, WfeRms},
};
use interface::{Tick, UID};
use skyangle::Conversion;

const N_STEP: usize = 50;
const M2_N_MODE: usize = 200;
const OIWFS: usize = 1;

type LtwsCentroid = Centroids<ZeroMean>;

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

    // OIWFS TIP-TILT SENSOR ...
    let oiwfs = Camera::builder().detector(Detector::default().n_px_imagelet(256));
    let mut oiwfs_centroids: Centroids = Centroids::try_from(&oiwfs)?;

    let mut oiwfs_tt_om_builder = OpticalModel::<Camera<OIWFS>>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("K"))
        .sensor(oiwfs);

    let mut calib_oiwfs_tt = <Centroids as Calibrate<GmtM2>>::calibrate(
        &((&oiwfs_tt_om_builder).into()),
        CalibrationMode::modes(M2_N_MODE, 1e-7)
            .start_from(2)
            .ends_at(3),
    )?;
    calib_oiwfs_tt.pseudoinverse();
    println!("{calib_oiwfs_tt}");

    oiwfs_tt_om_builder.initialize(&mut oiwfs_centroids);
    dbg!(oiwfs_centroids.n_valid_lenslets());

    let oiwfs_tt_om = oiwfs_tt_om_builder
        .atmosphere(atm_builder.clone())
        .build()?;
    println!("{oiwfs_tt_om}");
    // ... OIWFS TIP-TILT SENSOR

    // LTWS ...
    let ltws = Camera::builder()
        .lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(8))
        .lenslet_flux(0.75);
    let mut ltws_centroids: LtwsCentroid = Centroids::try_from(&ltws)?;

    let mut ltws_om_builder = OpticalModel::<Camera<OIWFS>>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder)
        .source(Source::builder().band("V"))
        .sensor(ltws);

    let mut calib_m2_modes = <LtwsCentroid as Calibrate<GmtM2>>::calibrate(
        &((&ltws_om_builder).into()),
        CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(4),
    )?;
    calib_m2_modes.pseudoinverse();
    println!("{calib_m2_modes}");

    ltws_om_builder.initialize(&mut ltws_centroids);

    let mut ltws_om = ltws_om_builder.atmosphere(atm_builder).build()?;
    println!("{ltws_om}");
    // ... LTWS

    let integrator = Integrator::new(M2_N_MODE * 7).gain(0.5);
    // let oiwfs_integrator = Integrator::new(M2_N_MODE * 7).gain(0.5);
    let adder = Operator::new("+");

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
                -> calib_m2_modes[Left<LtwsResidualAsmCmd>]${M2_N_MODE*7}
                    -> adder
        1: oiwfs_tt_om[Frame<Dev>]
            -> oiwfs_centroids[SensorData]
                -> calib_oiwfs_tt[Right<OiwfsResidualAsmCmd>]${M2_N_MODE*7}
                    -> adder[M2ASMAsmCommand]${M2_N_MODE*7}
        -> integrator[M2ASMAsmCommand]! -> ltws_om[WfeRms<-9>] -> print
        1: integrator[M2ASMAsmCommand]! -> oiwfs_tt_om[WfeRms<-9>] -> print
        1: ltws_om[SegmentWfeRms<-9>] -> print
        1: ltws_om[Wavefront]${481*481}



    );

    println!("{}", logging_1.lock().await);

    Ok(())
}

#[derive(UID)]
pub enum LtwsResidualAsmCmd {}

#[derive(UID)]
pub enum OiwfsResidualAsmCmd {}
