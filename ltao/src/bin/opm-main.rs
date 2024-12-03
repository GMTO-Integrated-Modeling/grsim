use std::{env, path::Path};

use crseo::{Atmosphere, FromBuilder, RayTracing};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    fun::Fun,
    gif,
    leftright::{Left as M1RBM, LeftRight, Right as ASMSmodes, Split},
    print::Print,
    select::Select,
    Integrator, Timer,
};
use gmt_dos_clients_crseo::{
    sensors::{builders::WaveSensorBuilder, WaveSensor},
    OpticalModel, OpticalModelBuilder,
};
use gmt_dos_clients_io::{
    gmt_m1::{assembly, M1ModeShapes, M1RigidBodyMotions},
    gmt_m2::asm::{M2ASMAsmCommand, M2ASMFaceSheetFigure},
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, Frame, Host, M2GlobalTipTilt, M2modes, SegmentPiston, SegmentWfeRms, SensorData,
        Wavefront, WfeRms,
    },
};
use gmt_dos_clients_servos::{
    asms_servo::ReferenceBody, AsmsServo, GmtFem, GmtM1, GmtM2, GmtServoMechanisms,
};
use interface::{filing::Filing, Tick, UID};
use ltao::{MergeAsmCommand, ModalToZonal, Model, Models, RxyPiston, M1_N_MODE, M2_N_MODE};
use skyangle::Conversion;

// const N_STEP: usize = 25;

const DFS_CAM_INT: usize = 5;
const DFS_FFT_INT: usize = 500;
const SH48_INT: usize = 2500;
const ACTUATOR_RATE: usize = 80;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("bin");
    env::set_var("DATA_REPO", &data_path);

    let sampling_frequency = 8000f64;
    let atm_sampling_frequency = 500.;
    // let atm_builder = Atmosphere::builder().ray_tracing(
    //     RayTracing::default()
    //         .field_size(10f64.from_arcmin())
    //         .duration(60.)
    //         .n_duration(15)
    //         .filepath("ltao-atmosphere.bin"),
    // );
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
    let ltws_processor = models.ltws().processor()?;
    let ltws_recon = models.ltws().reconstructor()?;
    println!("{ltws_recon}");
    let ltws_int = Integrator::new(M2_N_MODE * 7).gain(0.5);

    println!(
        r#"
-------
 OIWFS
-------"#
    );
    let oiwfs = models.oiwfs().build()?;
    println!("{oiwfs}");
    let oiwfs_processor = models.oiwfs().processor()?;
    let oiwfs_recon = models.oiwfs().reconstructor()?;
    let oiwfs_int = Integrator::new(2).gain(0.5);

    println!(
        r#"
-----
 DFS
-----"#
    );
    let dfs = models
        .dfs::<RxyPiston, DFS_CAM_INT, DFS_FFT_INT>()
        .build()?;
    println!("{dfs}");
    let dfs_processor = models
        .dfs::<RxyPiston, DFS_CAM_INT, DFS_FFT_INT>()
        .processor()?;
    let dfs_recon = models
        .dfs::<RxyPiston, DFS_CAM_INT, DFS_FFT_INT>()
        .reconstructor()?;
    println!("{dfs_recon}");
    let dfs_m1_rbm_int = Integrator::new(42).gain(0.1);
    let dfs_m2_bm_int = Integrator::new(7).gain(0.1);

    println!(
        r#"
------
 SH48
------"#
    );
    let sh48 = models.sh48::<SH48_INT>().build()?;
    println!("{sh48}");
    let sh48_processor = models.sh48::<SH48_INT>().processor()?;
    println!("{:?}", sh48_processor.n_valid_lenslets());
    let sh48_recon = models.sh48::<SH48_INT>().reconstructor()?;
    println!("{sh48_recon}");
    let sh48_m1_bm_int = Integrator::new(M1_N_MODE * 7).gain(0.1);

    let offaxis_om: OpticalModel<WaveSensor> = OpticalModelBuilder::<WaveSensorBuilder>::from(
        &models
            .dfs::<RxyPiston, DFS_CAM_INT, DFS_FFT_INT>()
            .builder(),
    )
    .build()?;
    println!("{offaxis_om}");

    println!(
        r#"
------------------
 SERVO-MECHANISMS
------------------"#
    );
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
    let modes2actuators = ModalToZonal::new().unwrap();

    // let cmd =
    //     geotrans::Mirror::<geotrans::M1>::tiptilt_2_rigidbodymotions((50f64.from_mas(), 0f64));
    // // let mut cmd = vec![0f64; 42];
    // // cmd[3] = 250f64.from_mas();
    // let m1_rbm = Signals::from((cmd.clone(), 20));

    // let print = Print::default();

    let oiwfs0_opd = gif::Gif::<f64>::new("oiwfs0_opd.gif", 512, 512)?;

    let add_m2_modes = MergeAsmCommand::new()?;
    // let t_xyz = Select::<f64>::new(0..3);
    let print = Print::default();
    let timer: Timer = Timer::new(20);
    actorscript!(
        #[model(name=ltws_oiwfs)]
        #[labels(ltws="LTWS",oiwfs="OIWFS")]
        // LTWS
        // 1: m1_rbm[M1RigidBodyMotions]
        16: timer[Tick]
            -> ltws[Frame<Dev>]!
                -> ltws_processor[SensorData]
                    -> ltws_recon[M2modes]
                        -> ltws_int[M2modes]
                            -> add_m2_modes[M2modes]
                                -> modes2actuators
                                // -> ltws
        // 16: add_m2_modes[M2ASMAsmCommand] -> oiwfs
        // OIWFS
        // 1: m1_rbm[M1RigidBodyMotions]
            // -> oiwfs[Frame<Dev>]!
        16: oiwfs[Frame<Dev>]!
                -> oiwfs_processor[SensorData]
                    -> oiwfs_recon[M2GlobalTipTilt]
                        -> oiwfs_int[M2GlobalTipTilt]
                            -> add_m2_modes //oiwfs
        // 1: oiwfs_int[M2GlobalTipTilt] -> ltws
        // 1: ltws[WfeRms<-9>] -> print
        16: oiwfs[WfeRms<-9>]$ -> print
        16: oiwfs[SegmentWfeRms<-9>]$ -> print
        16: oiwfs[SegmentPiston<-9>]$ -> print
        16: oiwfs[Wavefront]$ -> oiwfs0_opd

        1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> ltws
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> oiwfs
    );
    // oiwfs_opd.lock().await.save()?;

    // let diff_m1_rbm = Operator::new("+");
    // let add_m2_modes = Operator::new("+");
    let add_m2_modes = MergeAsmCommand::new()?;

    // let m1_rbm = Signals::from((cmd.clone(), 1600));

    let split_m12_rbm = LeftRight::<M1RbmASMSmodes, Split, M1RbmASMSmodes>::split_at(42);

    let idx: Vec<_> = (0..7)
        .flat_map(|i| (3..5).map(|j| i * 6 + j).collect::<Vec<_>>())
        .collect();
    let r_xyz = Select::<f64>::new(idx);
    let to_mas = Fun::new(|x: &Vec<f64>| x.iter().map(|x| x.to_mas()).collect::<Vec<_>>());
    // let idx: Vec<_> = (0..7).map(|i| i * M2_N_MODE).collect();
    // let t_z = Select::<f64>::new(idx);
    let to_nm = Fun::new(|x: &Vec<f64>| x.iter().map(|x| x * 1e9).collect::<Vec<_>>());

    // let sh48_sampler = Sampler::default();

    // let print = Print::default();
    let dfs_print = Print::default();
    let dfs_opd = gif::Frame::<f64>::new("dfs_opd.png", 512);
    let oiwfs_opd = gif::Frame::<f64>::new("oiwfs_opd.png", 512);
    // let sh48_frame = gif::Gif::<f32>::new("sh48_frame.gif", 48 * 8 * 3, 48 * 8)?;
    // let dfs_opd = gif::Gif::<f64>::new("dfs_opd.png", 512);
    let timer: Timer = Timer::new(SH48_INT * 10);
    actorscript!(
        #[model(name=ltws_oiwfs_dfs)]
        #[labels(ltws="GMT w/\n🌫  & LTWS",oiwfs="GMT w/\n🌫  & OIWFS",
            sh48="GMT w/\n🌫  & SH48",dfs="GMT w/\n🌫  & DFS")]
            // to_mas="To MAS",to_nm="To NM")]
        // LTWS
        // 1: m1_rbm[Left<M1RigidBodyMotions>]
        // -> diff_m1_rbm[M1RigidBodyMotions]
        16: timer[Tick]
            -> ltws[Frame<Dev>]!
                -> ltws_processor[SensorData]
                    -> ltws_recon[M2modes]
                        -> ltws_int[M2modes]
                            -> add_m2_modes[M2modes]
                                -> modes2actuators
        // 1: add_m2_modes[M2ASMAsmCommand] -> oiwfs
        // 1: add_m2_modes[M2ASMAsmCommand] -> dfs
        // 1: add_m2_modes[M2ASMAsmCommand] -> sh48
        // OIWFS
        // 1: diff_m1_rbm[M1RigidBodyMotions]
            // -> oiwfs[Frame<Dev>]!
        16: oiwfs[Frame<Dev>]!
                -> oiwfs_processor[M2GlobalTipTilt]
                    -> oiwfs_recon[M2GlobalTipTilt]
                        -> oiwfs_int[M2GlobalTipTilt]
                            -> add_m2_modes //oiwfs
        // SH48
        // 10: diff_m1_rbm[M1RigidBodyMotions]//${42}
            // -> sh48[Frame<Dev>]!
        2500: sh48[Frame<Dev>]!
                -> sh48_processor[SensorData]//${48*48*3*2}
                    -> sh48_recon[M1ModeShapes]
                            -> sh48_m1_bm_int
        // 1: sh48[Frame<Host>] -> sh48_frame
        1: sh48_m1_bm_int[M1ModeShapes]//${M1_N_MODE*7}
                                -> sh48
        1: sh48_m1_bm_int[M1ModeShapes] -> dfs
        1: sh48_m1_bm_int[M1ModeShapes] -> ltws
        1: sh48_m1_bm_int[M1ModeShapes] -> oiwfs
        // DFS
        // 10: diff_m1_rbm[M1RigidBodyMotions]
        //     -> dfs[DfsFftFrame<Dev>]!
        2500: dfs[DfsFftFrame<Dev>]!
                -> dfs_processor[Intercepts]//${36}
                    -> dfs_recon[M1RbmASMSmodes]
                        -> split_m12_rbm[M1RBM<M1RbmASMSmodes>]
                            -> dfs_m1_rbm_int
                                // -> diff_m1_rbm
        1: dfs_m1_rbm_int[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}
        // 1: dfs_m1_rbm_int[M1RigidBodyMotions] -> dfs
        // 1: dfs_m1_rbm_int[M1RigidBodyMotions] -> sh48
        // 1: dfs_m1_rbm_int[M1RigidBodyMotions] -> ltws
        // 1: dfs_m1_rbm_int[M1RigidBodyMotions] -> oiwfs
        2500: split_m12_rbm[ASMSmodes<M1RbmASMSmodes>]
            -> dfs_m2_bm_int
        1: dfs_m2_bm_int[SegmentPiston]
                -> add_m2_modes
        // 2500: split_m12_rbm[M1RBM<M1RbmASMSmodes>]
        //     -> r_xyz[M1Rxy]
        //         -> to_mas[M1Rxy]${14}
        //             // -> dfs_print
        // 2500: split_m12_rbm[ASMSmodes<M1RbmASMSmodes>]
        //     // -> t_z[M2Piston]
        //         -> to_nm[M2Piston]${7}
        //             // -> dfs_print

        // 10: ltws[WfeRms<-9>] -> dfs_print
        // 1: oiwfs[WfeRms<-9>]$
        // 1: oiwfs[SegmentWfeRms<-9>]$
        // 1: oiwfs[SegmentPiston<-9>]$
        160: oiwfs[WfeRms<-9>] -> dfs_print
        160: oiwfs[SegmentWfeRms<-9>] -> dfs_print
        160: oiwfs[SegmentPiston<-9>] -> dfs_print

        // 20: diff_m1_rbm[M1RigidBodyMotions] -> offaxis_om
        // 20: dfs_m1_rbm_int[M1RigidBodyMotions] -> offaxis_om
        // 20: sh48_m1_bm_int[M1ModeShapes] -> offaxis_om
        // 20: add_m2_modes[M2ASMAsmCommand] -> offaxis_om
        // 20: oiwfs_int[M2GlobalTipTilt] -> offaxis_om
        // 20: offaxis_om[Wavefront] -> dfs_opd
        // 1-1: oiwfs[Wavefront]$// -> oiwfs_opd
        // 1: dfs_m1_rbm_int[M1RigidBodyMotions]${42}
        // 1: dfs_m2_bm_int[SegmentPiston]${7}
        // 1: sh48_m1_bm_int[M1ModeShapes]${M1_N_MODE*7}~
        // 1: add_m2_modes[M2ASMAsmCommand]${M2_N_MODE *7}

        1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> ltws
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> oiwfs
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> dfs
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> sh48
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> ltws
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> oiwfs
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> dfs
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> sh48
        1: {gmt_servos::GmtFem}[M1ModeShapes] -> ltws
        1: {gmt_servos::GmtFem}[M1ModeShapes] -> oiwfs
        1: {gmt_servos::GmtFem}[M1ModeShapes] -> dfs
        1: {gmt_servos::GmtFem}[M1ModeShapes] -> sh48
    );

    // oiwfs_opd.lock().await.save()?;
    // dfs_opd.lock().await.save()?;

    Ok(())
}

#[derive(UID)]
pub enum M1Rxy {}
#[derive(UID)]
pub enum M2Piston {}
#[derive(UID)]
pub enum M1RbmASMSmodes {}
