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
    optics::{Dev, Frame, M2GlobalTipTilt, M2modes, SegmentWfeRms, SensorData, Wavefront, WfeRms},
};
use gmt_dos_clients_servos::{
    asms_servo::ReferenceBody, AsmsServo, GmtFem, GmtM2, GmtServoMechanisms, M1SegmentFigure,
};
// use gmt_fem::FEM;
use interface::{filing::Filing, Tick};
use ltao::{
    kernels::KernelFrame,
    m1_parameters::{M1_ACTUATOR_RATE, M1_N_RAW_MODE, RAW_BENDING_MODES},
    m2_parameters::{ASMS_INFLUENCE_FUNCTIONS, ASMS_MODES, ASM_N_ACTUATOR, M2_N_MODE},
    Ltws, MergeAsmCommand, ModalToZonal, Model, Models, Oiwfs,
};

const N_STEP: usize = 1601;
const ACTUATOR_RATE: usize = 80;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("servos_closed-loop_atmosphere");
    env::set_var("DATA_REPO", &data_path);

    let sampling_frequency = 8000f64;
    let atm_sampling_frequency = sampling_frequency;
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
    /* let gmt = Gmt::builder()
        .m1(RAW_BENDING_MODES, M1_N_RAW_MODE)
        .m2(ASMS_MODES, M2_N_MODE);

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
        .gmt(gmt.clone().m2(ASMS_INFLUENCE_FUNCTIONS, ASM_N_ACTUATOR))
        .sampling_frequency(atm_sampling_frequency)
        .atmosphere(atm_builder)
        .build()?;
    println!("{om}");

    let int = Integrator::new(M2_N_MODE * 7).gain(0.5); */

    /* // GMT Servomechanisms actors
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
    println!("{gmt_servos}"); */
    println!(
        r#"
------------------
 Servo-Mechanisms
------------------"#
    );
    // GMT Servomechanisms actors
    let gmt_servos = Sys::<GmtServoMechanisms<M1_ACTUATOR_RATE, 1>>::from_path_or_else(
        Path::new("/home/ubuntu/projects/grsim/ltao/src/bin").join(format!(
            "{}_servos.bin",
            Path::new(env!("FEM_REPO"))
                .file_name()
                .unwrap()
                .to_str()
                .unwrap()
        )),
        || {
            GmtServoMechanisms::<M1_ACTUATOR_RATE, 1>::new(
                sampling_frequency as f64,
                gmt_fem::FEM::from_env().unwrap(),
            )
            //.wind_loads(WindLoads::new())
            .m1_segment_figure(M1SegmentFigure::new())
            .asms_servo(
                AsmsServo::new()
                    .facesheet(Default::default())
                    .reference_body(ReferenceBody::new()),
            )
        },
    )?;
    println!("{gmt_servos}");

    let atm_builder = Atmosphere::builder()
        .single_turbulence_layer(0f32, Some(7f32), Some(0f32))
        .ray_tracing(
            RayTracing::default()
                .duration(5.)
                .n_duration(100)
                .filepath("atm_single_layer.bin"),
        );

    let models = Models::new().atmosphere(atm_sampling_frequency, atm_builder);

    println!(
        r#"
------
 LTWS
------"#
    );
    let ltws = models.ltws().build()?;
    println!("{ltws}");
    let ltws_int = Integrator::new(M2_N_MODE * 7).gain(0.5);
    let ltws_kernel = models.ltws().kernel(ltws_int)?;
    println!(
        r#"
-------
 OIWFS
-------"#
    );
    let oiwfs = models.oiwfs().build()?;
    println!("{oiwfs}");
    let oiwfs_int = Integrator::new(2).gain(0.5);
    let oiwfs_kernel = models.oiwfs().kernel(oiwfs_int)?;

    // Modal to zonal conversion
    let modes2actuators = ModalToZonal::asms().unwrap();
    let add_m2_modes = MergeAsmCommand::new()?;

    let wavefront = gif::Frame::<f64>::new("wavefront.png", ltws.source().pupil_sampling());

    let timer: Timer = Timer::new(N_STEP);
    let print = Print::default();
    actorscript!(
        #[model(name=closed_loop_atmosphere)]
        1: timer[Tick] -> ltws
            // -> om[Frame<Dev>]!
                // -> centroids[SensorData]
                    // -> recon[M2modes]
                        // -> int[M2modes]
            16: ltws[KernelFrame<Ltws>]!
                -> ltws_kernel[M2ASMAsmCommand]
                    -> add_m2_modes[M2modes]
                            -> modes2actuators
        16: oiwfs[KernelFrame<Oiwfs>]!
            -> oiwfs_kernel[M2GlobalTipTilt]
                -> add_m2_modes //oiwfs
        1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> ltws
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> oiwfs

        16: ltws[WfeRms<-9>] -> print
        16: oiwfs[SegmentWfeRms<-9>] -> print
        // 1: ltws[SegmentPiston<-9>]
        16: ltws[Wavefront] -> wavefront
    );

    wavefront.lock().await.save()?;

    Ok(())
}
