use std::{env, path::Path};

use crseo::{
    wavefrontsensor::PhaseSensor,
    //atmosphere,Atmosphere,
    FromBuilder,
    Gmt,
};
use gmt_dos_actors::actorscript;
//use gmt_dos_clients::{Tick, Timer};
//use gmt_dos_clients::Weight;
use gmt_dos_clients::Signals; //{OneSignal, Signal, Smooth};
use gmt_dos_clients_crseo::OpticalModel;
use gmt_dos_clients_io::{
    //cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{
        asm::{segment::FaceSheetFigure, M2ASMAsmCommand},
        M2RigidBodyMotions,
    },
    //mount::MountSetPoint,
    optics::Wavefront, //WfeRms,SegmentWfeRms
};
use gmt_dos_clients_servos::{asms_servo, AsmsServo, GmtFem, GmtM2, GmtServoMechanisms}; //GuideStar
                                                                                        //use gmt_dos_clients_scope::server::{Monitor, Scope};
                                                                                        //use gmt_dos_clients_windloads::CfdLoads;
use gmt_fem::FEM;
use interface::{units::MuM, Size, Write};
use matio_rs::MatFile;
use nalgebra as na;

const ACTUATOR_RATE: usize = 80;

/*
FEM_REPO=`pwd`/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/ cargo run --release
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let sim_sampling_frequency = 8000;
    //let optical_model_sampling_frequency = 1000;
    //let sim_duration = 5_usize; // second
    let n_step = 400; //sim_sampling_frequency * sim_duration;

    // $FEM_REPO-related variables
    let fem = FEM::from_env()?;
    let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set!");
    let fem_path = Path::new(&fem_var);

    // M2 modal basis
    let (m2_modes, n_mode) = ("ASM_IFs", 675);
    //let (m2_modes, n_mode) = ("M2_OrthoNormGS36p_KarhunenLoeveModes", 500);

    /*
    // Optical Model (GMT with 1 guide star on-axis)
    // Atmospher builder
    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(5f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );*/

    let optical_model: OpticalModel = OpticalModel::<PhaseSensor>::builder() //
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .build()?;

    // The CFD wind loads must be called next afer the FEM as it is modifying
    /*
    // the FEM CFDMountWindLoads inputs
    let cfd_loads = CfdLoads::foh(".", sim_sampling_frequency)
        .duration(sim_duration as f64)
        .mount(&mut fem, 0, None)
        .m1_segments()
        .m2_segments()
        .build()?;
    */
    // MOUNT SET POINT
    // let setpoint = Signals::new(3, n_step); //.channel(1, Signal::Constant(1f64.from_arcsec()));

    // ...
    //let stats = WavefrontStats::<1>::default();

    /*
    let sigmoid = OneSignal::try_from(Signals::new(1, n_step).channel(
        0,
        Signal::Sigmoid {
            amplitude: 1f64,
            sampling_frequency_hz: sim_sampling_frequency as f64,
        },
    ))?;
    */
    //let m1_smoother = Smooth::new();
    //let m2_smoother = Smooth::new();
    //let mount_smoother = Smooth::new();

    // let actuators = Signals::new(6 * 335 + 306, n_step);
    // let m1_rbm = Signals::new(6 * 7, n_step);

    // let m2_rbm: Signals<_> = Signals::new(6 * 7, n_step);
    // ASMS modal commands
    /*
    let mut modes = vec![vec![0f64; n_mode]; 7];
    [331,332,333,334,335,336,337,338,339,340,341,342,343,344,345]
        .into_iter()
        .for_each(|i| {
            modes[1][i - 1] = 1e-6;
            modes[3][i - 1] = 1e-6;
            modes[5][i - 1] = 1e-6;
        });        
    [117,220,225,244,313,321,463,472,474,526,547,578,603,605,631]
        .into_iter()
        .for_each(|i| {
            modes[0][i - 1] = 1e-6;
            modes[2][i - 1] = 1e-6;
            modes[4][i - 1] = 1e-6;
        });
    let asms_cmd: Signals<_> =
        Signals::from((modes.into_iter().flatten().collect::<Vec<f64>>(), n_step));
     */
    
    let mat_file = MatFile::load(&fem_path.join("KLmodesGS36p90.mat"))?;
    let kl_mat: Vec<na::DMatrix<f64>> = (1..=7)
        .map(|i| mat_file.var(format!("KL_{i}")).unwrap())
        .collect();
    let asms_mode_cmd_vec: Vec<usize> = vec![1, 8, 9, 1, 1, 1, 3];
    let asms_cmd_vec: Vec<_> = kl_mat
        .into_iter()
        .zip(asms_mode_cmd_vec.into_iter()) // Create the tuples (kl_mat[i], asms_mode_cmd_vec[i])
        .flat_map(|(kl_mat, i)| {
            kl_mat
                .column(i - 1)
                .as_slice()
                .iter()
                .map(|x| x * 1e-7)
                .collect::<Vec<f64>>()
        })
        .collect();
    dbg!(asms_cmd_vec.len());
    let asms_cmd: Signals<_> = Signals::from((asms_cmd_vec, n_step)); 
     
    //let asms_cmd: Signals<_> = Signals::new(675 * 7, n_step);

    let gmt_servos =
        GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
            .asms_servo(
                AsmsServo::new().facesheet(
                    asms_servo::Facesheet::new()
                        .transforms(fem_path.join("asms_Pmats_20230131_1605.mat"), "PmatT"),
                ),
            )
            .build()?;

    // let gmt_servos =
    //     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
    //         .asms_servo(AsmsServo::new().facesheet(Default::default()))
    //         .build()?;

    /*
    // Scopes definition
    let mut monitor = Monitor::new();
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;
    */
    actorscript! (
    // 1: setpoint[MountSetPoint] -> {gmt_servos::GmtMount}

    /*
    1: cfd_loads[CFDM1WindLoads] -> m1_smoother
    1: sigmoid[Weight] -> m1_smoother[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: cfd_loads[CFDM2WindLoads] -> m2_smoother
    1: sigmoid[Weight] -> m2_smoother[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: cfd_loads[CFDMountWindLoads] -> mount_smoother
    1: sigmoid[Weight] -> mount_smoother[CFDMountWindLoads] -> {gmt_servos::GmtFem}

    // 1: m1_rbm[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}
    // 1: actuators[assembly::M1ActuatorCommandForces] -> {gmt_servos::GmtM1}
    // 1: m2_rbm[M2RigidBodyMotions]-> {gmt_servos::GmtM2Hex}
    */
    1: asms_cmd[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
    8: optical_model
    1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> optical_model
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> optical_model
    1: {gmt_servos::GmtFem}[FaceSheetFigure<1>]${675} -> optical_model
    1: {gmt_servos::GmtFem}[FaceSheetFigure<2>] -> optical_model
    1: {gmt_servos::GmtFem}[FaceSheetFigure<3>] -> optical_model
    1: {gmt_servos::GmtFem}[FaceSheetFigure<4>] -> optical_model
    1: {gmt_servos::GmtFem}[FaceSheetFigure<5>] -> optical_model
    1: {gmt_servos::GmtFem}[FaceSheetFigure<6>] -> optical_model
    1: {gmt_servos::GmtFem}[FaceSheetFigure<7>] -> optical_model

    );

    // let metronome: Timer = Timer::new(0);
    // actorscript!(
    //     #[model(name=wavefront)]
    //     1: metronome[Tick] -> optical_model[Wavefront]!$
    // );

    let mut opm = optical_model.lock().await;
    let phase = <OpticalModel as Write<MuM<Wavefront>>>::write(&mut opm).unwrap();
    let n_px = (<OpticalModel as Size<Wavefront>>::len(&mut opm) as f64).sqrt() as usize;

    let _: complot::Heatmap = ((phase.as_arc().as_slice(), (n_px, n_px)), None).into();

    Ok(())
}
