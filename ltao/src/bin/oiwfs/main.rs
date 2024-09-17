use std::path::Path;

use crseo::{gmt::GmtM2, imaging::Detector, FromBuilder, Gmt, Source};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{print::Print, Integrator, Signal, Signals, Timer};
use gmt_dos_clients_crseo::{
    calibration::{Calibrate, CalibrationMode},
    sensors::Camera,
    Centroids, DeviceInitialize, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{Dev, Frame, WfeRms},
    M12RigidBodyMotions,
};
use interface::{Tick, UID};
use skyangle::Conversion;

const N_STEP: usize = 10;
const M2_N_MODE: usize = 3;
const OIWFS: usize = 1;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    std::env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("bin")
            .join("oiwfs"),
    );

    let sampling_frequency = 1000f64;

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
        CalibrationMode::modes(M2_N_MODE, 1e-8)
            // .start_from(2)
            .ends_at(3),
    )?;
    calib_oiwfs_tt.pseudoinverse();
    println!("{calib_oiwfs_tt}");

    oiwfs_tt_om_builder.initialize(&mut oiwfs_centroids);
    dbg!(oiwfs_centroids.n_valid_lenslets());

    let oiwfs_tt_om = oiwfs_tt_om_builder.build()?;
    println!("{oiwfs_tt_om}");
    // ... OIWFS TIP-TILT SENSOR

    let integrator = Integrator::new(M2_N_MODE * 7).gain(0.5);

    let m1_rbm = Signals::new(42, N_STEP).channel(3, 100f64.from_mas());

    let print = Print::default();
    actorscript!(
        #[model(name=oiwfs)]
        1: m1_rbm[M1RigidBodyMotions]
        -> oiwfs_tt_om[Frame<Dev>]!
            -> oiwfs_centroids[OiwfsData]
                -> calib_oiwfs_tt[OiwfsResidualAsmCmd]
                    -> integrator[M2ASMAsmCommand]
                        -> oiwfs_tt_om[WfeRms<-9>] -> print
    );

    Ok(())
}

#[derive(UID)]
pub enum OiwfsResidualAsmCmd {}

#[derive(UID)]
pub enum OiwfsData {}
