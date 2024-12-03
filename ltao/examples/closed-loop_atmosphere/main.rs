use std::{env, path::Path};

use crseo::{gmt::GmtM2, imaging::LensletArray, Atmosphere, FromBuilder, Gmt, RayTracing};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif, print::Print, Integrator, Timer};
use gmt_dos_clients_crseo::{
    calibration::{Calibration, CalibrationMode},
    centroiding::CentroidsProcessing,
    sensors::{Camera, NoSensor},
    DeviceInitialize, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{Dev, Frame, SegmentWfeRms, SensorData, Wavefront, WfeRms},
};
use interface::Tick;

const M2_N_MODE: usize = 224;
const N_STEP: usize = 201;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("closed-loop_atmosphere");
    env::set_var("DATA_REPO", &data_path);

    // let gmt = Gmt::builder().m2("Karhunen-Loeve", M2_N_MODE);
    let gmt = Gmt::builder().m2("asms_eigenmodes_wz123", M2_N_MODE);

    let sampling_frequency = 500f64;
    let atm_builder = Atmosphere::builder()
        .single_turbulence_layer(0f32, Some(7f32), Some(0f32))
        .ray_tracing(
            RayTracing::default().duration(1.), // .filepath("atm_single_layer.bin"),
        );

    let sensor =
        Camera::builder().lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(8));

    let mut centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sensor)?;

    let om = OpticalModel::<Camera>::builder()
        .gmt(gmt.clone())
        .sensor(sensor);

    let mut recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(&om).into(),
        CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(2),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    om.initialize(&mut centroids);

    let om = om
        .sampling_frequency(sampling_frequency)
        .atmosphere(atm_builder.clone())
        .build()?;

    let om_ref = OpticalModel::<NoSensor>::builder()
        .gmt(gmt.clone())
        .sampling_frequency(sampling_frequency)
        .atmosphere(atm_builder)
        .build()?;

    let int = Integrator::new(M2_N_MODE * 7).gain(0.5);

    let wavefront = gif::Frame::<f64>::new("wavefront.png", om.source().pupil_sampling());

    let timer: Timer = Timer::new(N_STEP);
    let print = Print::default();
    actorscript!(
        #[model(name=closed_loop_atmosphere)]
        1: timer[Tick]
            -> om[Frame<Dev>]!
                -> centroids[SensorData]
                    -> recon[M2ASMAsmCommand]
                        -> int[M2ASMAsmCommand] -> om
        1: om[WfeRms<-9>] -> print
        1: om_ref[WfeRms<-9>] -> print
        1: om[SegmentWfeRms<-9>] -> print
        // 1: om[SegmentPiston<-9>]
        100: om[Wavefront] -> wavefront
    );

    wavefront.lock().await.save()?;

    Ok(())
}
