use std::{env, path::Path};

use crseo::{gmt, imaging::LensletArray, Atmosphere, FromBuilder, Gmt, RayTracing};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{gif, print::Print, Integrator, Timer};
use gmt_dos_clients_crseo::{
    calibration::{Calibration, CalibrationMode},
    centroiding::CentroidsProcessing,
    sensors::Camera,
    DeviceInitialize, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m2::asm::{M2ASMAsmCommand, M2ASMFaceSheetFigure},
    optics::{Dev, Frame, M2modes, SegmentWfeRms, SensorData, Wavefront, WfeRms},
};
use gmt_dos_clients_servos::{
    asms_servo::ReferenceBody, AsmsServo, GmtFem, GmtM2, GmtServoMechanisms,
};
// use gmt_fem::FEM;
use interface::{filing::Filing, Tick};
use ltao::{ModalToZonal, M2_N_MODE, N_ACTUATOR};

const N_STEP: usize = 101;
const ACTUATOR_RATE: usize = 80;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("asms_closed-loop_atmosphere");
    env::set_var("DATA_REPO", &data_path);

    let sampling_frequency = 8000f64;
    let atm_sampling_frequency = 500.;
    // let sim_sampling_frequency = 8000;
    // let sim_duration = 1_usize; // second
    // let n_step = sim_sampling_frequency * sim_duration;

    // let mut fem = gmt_fem::FEM::from_env().unwrap();

    // let asms = ASMS::<1>::new(&mut fem)?.build()?;
    // let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
    //     .sampling(sim_sampling_frequency as f64)
    //     .proportional_damping(2. / 100.)
    //     .including_asms(Some(vec![1, 2, 3, 4, 5, 6, 7]), None, None)?
    //     .outs_by_name((1..=7).map(|i| format!("M2_segment_{i}_axial_d")).collect())?
    //     .build()?;
    // println!("{plant}");
    // // c            self.outs_by_name((1..=7).map(|i| format!("M2_segment_{i}_axial_d")).collect())?

    // let gmt = Gmt::builder().m2("Karhunen-Loeve", M2_N_MODE);
    let gmt = Gmt::builder().m2("asms_eigenmodes_wz123", M2_N_MODE);

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

    let mut recon = <CentroidsProcessing as Calibration<gmt::GmtM2>>::calibrate(
        &(&om).into(),
        CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(2),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    om.initialize(&mut centroids);

    let om = om
        .gmt(gmt.clone().m2("asms_ifs_gmt-fem", N_ACTUATOR))
        .sampling_frequency(atm_sampling_frequency)
        .atmosphere(atm_builder)
        .build()?;
    println!("{om}");

    let int = Integrator::new(M2_N_MODE * 7).gain(0.5);

    // GMT Servomechanisms actors
    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path_or_else(
        // Path::new(env!("FEM_REPO")).join("servos.bin"),
        Path::new(".").join("servos.bin"),
        || {
            GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(
                sampling_frequency as f64,
                gmt_fem::FEM::from_env().unwrap(),
            )
            //.wind_loads(WindLoads::new())
            .asms_servo(
                AsmsServo::new()
                    .facesheet(Default::default())
                    .reference_body(ReferenceBody::new()),
            )
        },
    )?;
    println!("{gmt_servos}");

    // Modal to zonal conversion
    let modes2actuators = ModalToZonal::asms().unwrap();

    let wavefront = gif::Frame::<f64>::new("wavefront.png", om.source().pupil_sampling());

    let timer: Timer = Timer::new(N_STEP);
    let print = Print::default();
    actorscript!(
        #[model(name=closed_loop_atmosphere)]
        16: timer[Tick]
            -> om[Frame<Dev>]!
                -> centroids[SensorData]
                    -> recon[M2modes]
                        -> int[M2modes]
                            -> modes2actuators
        1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> om

        16: om[WfeRms<-9>] -> print
        16: om[SegmentWfeRms<-9>] -> print
        // 1: om[SegmentPiston<-9>]
        1600: om[Wavefront] -> wavefront
    );

    wavefront.lock().await.save()?;

    Ok(())
}
