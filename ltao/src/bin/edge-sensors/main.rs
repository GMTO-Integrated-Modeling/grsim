use std::{env, path::Path};

use crseo::{Atmosphere, FromBuilder, RayTracing};
use edge_sensors::{
    AsmsToHexOffload, EdgeSensorsFeedForward, HexToRbm, M1EdgeSensorsAsRbms, M1EdgeSensorsToRbm,
    M2EdgeSensorsToRbm, RbmToShell,
};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    gif,
    leftright::{Left, LeftRight, Right, Split},
    low_pass_filter::LowPassFilter,
    operator::{self, Operator},
    print::Print,
    Integrator, Timer,
};
use gmt_dos_clients_crseo::{
    sensors::{builders::WaveSensorBuilder, WaveSensor},
    OpticalModel, OpticalModelBuilder,
};
use gmt_dos_clients_io::{
    gmt_fem::outputs::MCM2SmHexD,
    gmt_m1::{
        assembly::{M1ActuatorCommandForces, M1ModeCoefficients},
        M1ModeShapes, M1RigidBodyMotions,
    },
    gmt_m2::{
        asm::{M2ASMAsmCommand, M2ASMFaceSheetFigure, M2ASMVoiceCoilsMotion},
        M2EdgeSensors, M2RigidBodyMotions,
    },
    optics::{M2GlobalTipTilt, M2modes, SegmentPiston, SegmentWfeRms, Wavefront, WfeRms},
};
use gmt_dos_clients_servos::{
    asms_servo::ReferenceBody, AsmsServo, EdgeSensors, GmtFem, GmtM1, GmtM2, GmtM2Hex,
    GmtServoMechanisms, M1SegmentFigure,
};
use interface::{filing::Filing, Data, Read, Tick, Update, Write, UID};
use ltao::{
    agws_parameters::{DFS_CAM_INT, DFS_FFT_INT, SH48_INT},
    kernels::KernelFrame,
    m1_parameters::{M1_ACTUATOR_RATE, M1_N_MODE},
    m2_parameters::M2_N_MODE,
    Dfs, Ltws, M1BendingModes, M1RbmM2modes, MergeAsmCommand, ModalToZonal, Model, Models, Oiwfs,
    RxyPiston, Sh48,
};
use matio_rs::MatFile;
use nalgebra as na;

// const N_STEP: usize = 25;
const ASM_LPF_GAIN: f64 = 0.05;
const ASM_OFFLOAD_GAIN: f64 = 1. / 2000f64;
const ASM_OFFLOAD_LEAK: f64 = 0.8;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("bin")
        .join("edge-sensors");
    env::set_var("DATA_REPO", &data_repo);

    let sampling_frequency = 8000f64;
    let atm_sampling_frequency = sampling_frequency;
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
    let dfs_int = Integrator::new(49).gain(0.1);
    let dfs_kernel = models
        .dfs::<RxyPiston, DFS_CAM_INT, DFS_FFT_INT>()
        .kernel(dfs_int)?;

    println!(
        r#"
------
 SH48
------"#
    );
    let sh48 = models.sh48::<SH48_INT>().build()?;
    println!("{sh48}");
    let sh48_m1_bm_int = Integrator::new(M1_N_MODE * 7).gain(0.1);
    let sh48_kernel = models.sh48::<SH48_INT>().kernel(sh48_m1_bm_int)?;

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
 Servo-Mechanisms
------------------"#
    );
    // GMT Servomechanisms actors
    // EDGE SENSORS
    //  * M1 EDGE SENSORS NODES
    let es_nodes_2_data: na::DMatrix<f64> =
        MatFile::load(data_repo.join("M1_edge_sensor_conversion.mat"))?.var("A1")?;
    //  * EDGE SENSORS TO RIGID-BODY MOTIONS TRANSFORM (M1 & M2)
    let es_2_m1_rbm = {
        let mat = MatFile::load(data_repo.join("m12_r_es.mat"))?;
        let m1_es_recon: na::DMatrix<f64> = mat.var("m1_r_es")?;
        m1_es_recon.insert_rows(36, 6, 0f64) * es_nodes_2_data
    };
    dbg!(es_2_m1_rbm.shape());
    let gmt_servos = Sys::<GmtServoMechanisms<M1_ACTUATOR_RATE, 1>>::from_data_repo_or_else(
        // Path::new(env!("FEM_REPO")).join("servos.bin"),
        format!(
            "{}_servos_edge-sensors.bin",
            Path::new(env!("FEM_REPO"))
                .file_name()
                .unwrap()
                .to_str()
                .unwrap()
        ),
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
            .edge_sensors(EdgeSensors::both().m1_with(es_2_m1_rbm))
        },
    )?;
    println!("{gmt_servos}");

    // Voice coils displacements to rigid body motions
    let asms_to_pos =
        Sys::new(AsmsToHexOffload::leaky(ASM_OFFLOAD_GAIN, ASM_OFFLOAD_LEAK)?).build()?;
    // Rigid body motions to facesheet displacements
    let edge_sensors_feedfwd = Sys::new(EdgeSensorsFeedForward::new(ASM_LPF_GAIN)?).build()?;

    // let cmd =
    //     geotrans::Mirror::<geotrans::M1>::tiptilt_2_rigidbodymotions((50f64.from_mas(), 0f64));
    // // let mut cmd = vec![0f64; 42];
    // // cmd[3] = 250f64.from_mas();
    // let m1_rbm = Signals::from((cmd.clone(), 20));

    // let print = Print::default();

    // let mom = Sys::new(MetaOpticalModel::<DFS_CAM_INT, DFS_FFT_INT, SH48_INT>::new()?).build()?;

    // Modal to zonal conversion
    let modes2actuators = ModalToZonal::asms().unwrap();

    let oiwfs0_opd = gif::Gif::<f64>::new("oiwfs0_opd.gif", 512, 512)?;
    let add_m2_modes = MergeAsmCommand::new()?;
    let m1_bms = M1BendingModes::new()?;
    let modes_to_forces = ModalToZonal::m1().unwrap();

    // let t_xyz = Select::<f64>::new(0..3);
    let print = Print::default();
    let timer: Timer = Timer::new(20 * 16);
    actorscript!(
        #[model(name=ltws_oiwfs)]
        #[labels(ltws="LTWS",oiwfs="OIWFS")]
        // LTWS
        // 1: m1_rbm[M1RigidBodyMotions]
        1: timer[Tick] -> ltws
        16: ltws[KernelFrame<Ltws>]!
            -> ltws_kernel[M2ASMAsmCommand]
                -> add_m2_modes[M2modes]
                    -> modes2actuators// ltws
        // 16: add_m2_modes[M2ASMAsmCommand] -> oiwfs//oiwfs
        // OIWFS
        16: oiwfs[KernelFrame<Oiwfs>]!
            -> oiwfs_kernel[M2GlobalTipTilt]
                -> add_m2_modes //oiwfs

        1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M1ModeShapes]
            -> m1_bms[M1ModeCoefficients]

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> ltws
        1: m1_bms[M1ModeCoefficients] -> ltws
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> ltws
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> ltws

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> oiwfs
        1: m1_bms[M1ModeCoefficients] -> oiwfs
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> oiwfs
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> oiwfs

        16: oiwfs[WfeRms<-9>]${1} -> print
        16: oiwfs[SegmentWfeRms<-9>]${7} -> print
        16: oiwfs[SegmentPiston<-9>]${7} -> print
        16: oiwfs[Wavefront]$ -> oiwfs0_opd
    );
    // oiwfs_opd.lock().await.save()?;

    // let diff_m1_rbm = Operator::new("+");
    // let add_m2_modes = Operator::new("+");
    let add_m2_modes = MergeAsmCommand::new()?;

    // let m1_rbm = Signals::from((cmd.clone(), 1600));

    let split_m12_rbm = DfsSplit(LeftRight::<M1RbmM2modes, Split, M1RbmM2modes>::split_at(42));

    // let idx: Vec<_> = (0..7)
    // .flat_map(|i| (3..5).map(|j| i * 6 + j).collect::<Vec<_>>())
    // .collect();
    // let r_xyz = Select::<f64>::new(idx);
    // let to_mas = Fun::new(|x: &Vec<f64>| x.iter().map(|x| x.to_mas()).collect::<Vec<_>>());
    // let idx: Vec<_> = (0..7).map(|i| i * M2_N_MODE).collect();
    // let t_z = Select::<f64>::new(idx);
    // let to_nm = Fun::new(|x: &Vec<f64>| x.iter().map(|x| x * 1e9).collect::<Vec<_>>());

    // let sh48_sampler = Sampler::default();

    // let print = Print::default();
    let dfs_print = Print::default();
    // let dfs_opd = gif::Frame::<f64>::new("dfs_opd.png", 512);
    // let oiwfs_opd = gif::Frame::<f64>::new("oiwfs_opd.png", 512);
    // let sh48_frame = gif::Gif::<f32>::new("sh48_frame.gif", 48 * 8 * 3, 48 * 8)?;
    // let dfs_opd = gif::Gif::<f64>::new("dfs_opd.png", 512);
    let timer: Timer = Timer::new(SH48_INT * 10 * 16);
    type Operatorf64 = Operator<f64>;
    type LowPassFilterf64 = LowPassFilter<f64>;
    actorscript!(
        #[model(name=ltws_oiwfs_dfs)]
        #[labels(ltws="GMT w/\n🌫  & LTWS",oiwfs="GMT w/\n🌫  & OIWFS",
            sh48="GMT w/\n🌫  & SH48",dfs="GMT w/\n🌫  & DFS")]
            // to_mas="To MAS",to_nm="To NM")]
        // LTWS
        1: timer[Tick] -> ltws
        16: ltws[KernelFrame<Ltws>]!
            -> ltws_kernel[M2ASMAsmCommand]
                -> add_m2_modes[M2modes]
                    -> modes2actuators // ltws
        // 1: add_m2_modes[M2ASMAsmCommand] -> oiwfs
        // 1: add_m2_modes[M2ASMAsmCommand] -> dfs
        // 1: add_m2_modes[M2ASMAsmCommand] -> sh48
        // OIWFS
        16: oiwfs[KernelFrame<Oiwfs>]!
            -> oiwfs_kernel[M2GlobalTipTilt]
                -> add_m2_modes //oiwfs

        1: {gmt_servos::GmtFem}[M1ModeShapes]
            -> m1_bms[M1ModeCoefficients]
        // SH48
        40_000: sh48[KernelFrame<Sh48<SH48_INT>>]!
            -> sh48_kernel
        // 1: sh48[Frame<Host>] -> sh48_frame
        1: sh48_kernel[M1ModeShapes]//${M1_N_MODE*7}
            // -> sh48
         -> modes_to_forces[M1ActuatorCommandForces] -> {gmt_servos::GmtM1}
        // 1: sh48_kernel[M1ModeShapes] -> dfs
        // 1: sh48_kernel[M1ModeShapes] -> ltws
        // 1: sh48_kernel[M1ModeShapes] -> oiwfs
        // DFS
        40_000: dfs[KernelFrame<Dfs<RxyPiston,DFS_CAM_INT,DFS_FFT_INT>>]!
            -> dfs_kernel[M1RbmM2modes]
                -> split_m12_rbm
        1: split_m12_rbm[M1RigidBodyMotions] -> {gmt_servos::GmtM1} //dfs
        // 1: split_m12_rbm[M1RigidBodyMotions] -> sh48
        // 1: split_m12_rbm[M1RigidBodyMotions] -> ltws
        // 1: split_m12_rbm[M1RigidBodyMotions] -> oiwfs
        1: split_m12_rbm[SegmentPiston] -> add_m2_modes

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> ltws
        1: m1_bms[M1ModeCoefficients] -> ltws
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> ltws
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> ltws

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> oiwfs
        1: m1_bms[M1ModeCoefficients] -> oiwfs
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> oiwfs
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> oiwfs

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> dfs
        1: m1_bms[M1ModeCoefficients] -> dfs
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> dfs
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> dfs

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> sh48
        1: m1_bms[M1ModeCoefficients] -> sh48
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> sh48
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> sh48

        // send the edge sensors data to the ASMS off-loading algorithm
        1: {gmt_servos::GmtFem}[M2EdgeSensors]! -> {edge_sensors_feedfwd::M2EdgeSensorsToRbm}
        // send the reference body RBMS to the ASMS off-loading algorithm
        1: {gmt_servos::GmtFem}[MCM2SmHexD]! -> {edge_sensors_feedfwd::HexToRbm}
        1: {gmt_servos::GmtFem}[M1EdgeSensorsAsRbms]! -> {edge_sensors_feedfwd::RbmToShell}
        // send the ASMS command (actuator displacement) to the ASMS controller
        1: modes2actuators[operator::Left<M2ASMAsmCommand>] -> {edge_sensors_feedfwd::Operatorf64}
        1: {edge_sensors_feedfwd::LowPassFilterf64}[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        // 1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}

        // read the voice coil displacement from the FEM
        1: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion]!
            // transfrom them to rigid body motions (RBMS)
            -> {asms_to_pos}[M2RigidBodyMotions] -> {gmt_servos::GmtM2Hex}

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
        /* 1: split_m12_rbm[M1RigidBodyMotions]${42}
        1: split_m12_rbm[SegmentPiston]${7}
        1: sh48_kernel[M1ModeShapes]${M1_N_MODE*7}
        1: add_m2_modes[M2ASMAsmCommand]${M2_N_MODE *7} */
    );

    // oiwfs_opd.lock().await.save()?;
    // dfs_opd.lock().await.save()?;

    Ok(())
}

#[derive(UID)]
pub enum M1Rxy {}
#[derive(UID)]
pub enum M2Piston {}

pub struct DfsSplit(LeftRight<M1RbmM2modes, Split, M1RbmM2modes>);
impl Update for DfsSplit {}
impl Read<M1RbmM2modes> for DfsSplit {
    fn read(&mut self, data: Data<M1RbmM2modes>) {
        <LeftRight<M1RbmM2modes, Split, M1RbmM2modes> as Read<M1RbmM2modes>>::read(
            &mut self.0,
            data,
        )
    }
}
impl Write<M1RigidBodyMotions> for DfsSplit {
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        <LeftRight<M1RbmM2modes, Split, M1RbmM2modes> as Write<Left<M1RbmM2modes>>>::write(
            &mut self.0,
        )
        .map(|data| data.transmute::<M1RigidBodyMotions>())
    }
}
impl Write<SegmentPiston> for DfsSplit {
    fn write(&mut self) -> Option<Data<SegmentPiston>> {
        <LeftRight<M1RbmM2modes, Split, M1RbmM2modes> as Write<Right<M1RbmM2modes>>>::write(
            &mut self.0,
        )
        .map(|data| data.transmute::<SegmentPiston>())
    }
}
