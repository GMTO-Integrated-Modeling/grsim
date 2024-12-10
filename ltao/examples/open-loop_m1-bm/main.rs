use std::{env, path::Path};

use crseo::{FromBuilder, Gmt};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif, Signals, Timer};
use gmt_dos_clients_crseo::{
    sensors::{Camera, NoSensor},
    OpticalModel,
};
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::outputs::M1Segment1AxialD,
    gmt_m1::{
        assembly::{self, M1ActuatorCommandForces},
        segment::{ActuatorCommandForces, BendingModes},
        M1ModeShapes, M1RigidBodyMotions,
    },
    mount::{MountEncoders, MountTorques},
};
use gmt_dos_clients_mount::Mount;
// use gmt_fem::FEM;
use gmt_dos_clients_fem::fem_io::actors_outputs::OSSM1Lcl;
use ltao::{
    m2_parameters::{ASM_N_ACTUATOR, M1_N_MODE},
    M1BendingModes, ModalToZonal,
};

const N_STEP: usize = 16000;
const ACTUATOR_RATE: usize = 80;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("open-loop_m1-bm");
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
    let gmt = Gmt::builder()
        .m1("m1_bm_gmt-fem", M1_N_MODE)
        .m2("asms_ifs_gmt-fem", ASM_N_ACTUATOR);

    // let atm_builder = Atmosphere::builder()
    //     .single_turbulence_layer(0f32, Some(7f32), Some(0f32))
    //     .ray_tracing(
    //         RayTracing::default().duration(1.), // .filepath("atm_single_layer.bin"),
    //     );

    // let sensor =
    //     Camera::builder().lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(8));

    // let mut centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sensor)?;

    let om = OpticalModel::<Camera>::builder().gmt(gmt.clone());
    // .sensor(sensor);

    // let mut recon = <CentroidsProcessing as Calibration<gmt::GmtM2>>::calibrate(
    //     &(&om).into(),
    //     CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(2),
    // )?;
    // recon.pseudoinverse();
    // println!("{recon}");

    // om.initialize(&mut centroids);

    let om = OpticalModel::<NoSensor>::builder()
        .gmt(gmt.clone())
        // .sampling_frequency(atm_sampling_frequency)
        // .atmosphere(atm_builder)
        .build()?;
    println!("{om}");

    let mut fem = gmt_fem::FEM::from_env().unwrap();
    println!("{fem}");
    // let int = Integrator::new(M2_N_MODE * 7).gain(0.5);
    let m1_calibration = gmt_dos_clients_m1_ctrl::Calibration::new(&mut fem);
    let m1 = gmt_dos_clients_m1_ctrl::M1::<ACTUATOR_RATE>::new(&m1_calibration)?;

    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .truncate_hankel_singular_values(1e-5)
        .hankel_frequency_lower_bound(50.)
        // .use_static_gain_compensation()
        .including_mount()
        .including_m1(Some(vec![1, 2, 3, 4, 5, 6, 7]))?
        .outs_by_name((1..=7).map(|i| format!("M1_segment_{i}_axial_d")).collect())?
        .outs::<OSSM1Lcl>()
        .build()?;
    println!("{plant}");

    // Modal to zonal conversion
    let modes2actuators = M1BendingModes::new(
        "/home/ubuntu/projects/dos-actors/clients/fem/m1s1_sms.pkl",
        M1_N_MODE,
    )
    .unwrap();
    let n = om.source().pupil_sampling();
    // let wavefront = gif::Gif::<f64>::new("wavefront.png", n, n);

    let mut m1_bm = vec![0f64; M1_N_MODE];
    m1_bm.chunks_mut(M1_N_MODE).for_each(|bm| bm[0] = 1e-5);
    let signal = Signals::from((m1_bm, N_STEP));

    // let timer: Timer = Timer::new(N_STEP);
    // let print = Print::default();
    /* actorscript!(
    // 1: timer[Tick]
        // -> om//[Frame<Dev>]!
            // -> centroids[SensorData]
                // -> recon[M2modes]
                    // -> int[M2modes]
                        // -> modes2actuators
        1: signal[BendingModes<1>]
            -> modes2actuators[ActuatorCommandForces<1>]
                 -> {m1}
                     // -> plant[M1ModeShapes]${602*6+579}
            // -> {m1}[assembly::M1HardpointsForces]${84}
        // 1: om[WfeRms<-9>] -> print
        // 1: om[SegmentWfeRms<-9>] -> print
        // 1: om[SegmentPiston<-9>]
        // 1600: om[Wavefront] -> wavefront
        1: {m1}[assembly::M1HardpointsForces]
                -> plant[assembly::M1HardpointsMotion]! -> {m1}
        1: {m1}[assembly::M1ActuatorAppliedForces] -> plant

        1: plant[M1Segment1AxialD]!${602} -> modes2actuators[BendingModes<1>]${M1_N_MODE}
        1: plant[M1RigidBodyMotions]${42}
    ); */

    // wavefront.lock().await.save()?;

    // MOUNT CONTROL
    let mount = Mount::new();

    let rbm = Signals::new(6 * 7, N_STEP);

    actorscript! {
    1:  mount[MountTorques] -> plant[MountEncoders]! -> mount

    1: rbm[assembly::M1RigidBodyMotions]
        -> {m1}[assembly::M1HardpointsForces]
            -> plant[assembly::M1HardpointsMotion]! -> {m1}
    1: plant[M1Segment1AxialD]!${602}
    // 1: actuators[assembly::M1ActuatorCommandForces]
            // -> {m1}[assembly::M1ActuatorAppliedForces] -> plant

    // 1: plant[M1RigidBodyMotions] -> lom
    // 1: plant[M2RigidBodyMotions] -> lom

    }

    Ok(())
}
