use std::{env, path::Path};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    gif,
    leftright::{Left as M1RBM, LeftRight, Right as M2modes, Split},
    print::Print,
    Integrator, Timer,
};
use gmt_dos_clients_crseo::{
    sensors::{builders::WaveSensorBuilder, WaveSensor},
    OpticalModel, OpticalModelBuilder,
};
use gmt_dos_clients_io::{
    gmt_m1::{M1ModeShapes, M1RigidBodyMotions},
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, Frame, M2GlobalTipTilt, SegmentPiston, SegmentWfeRms, SensorData, Wavefront, WfeRms,
    },
};
use interface::{Tick, UID};
use ltao_tests::{MergeAsmCommand, Model, Models, RxyPiston, M1_N_MODE, M2_N_MODE};

// const N_STEP: usize = 25;

const C: usize = 10;
const F: usize = 10;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("closed-loop_ltws-oiwfs");
    env::set_var("DATA_REPO", &data_path);

    let models = Models::new();

    let ltws = models.ltws().build()?;
    println!("{ltws}");
    let ltws_processor = models.ltws().processor()?;
    let ltws_recon = models.ltws().reconstructor()?;
    let ltws_int = Integrator::new(M2_N_MODE * 7).gain(0.5);

    let oiwfs = models.oiwfs().build()?;
    println!("{oiwfs}");
    let oiwfs_processor = models.oiwfs().processor()?;
    let oiwfs_recon = models.oiwfs().reconstructor()?;
    let oiwfs_int = Integrator::new(2).gain(0.5);

    let dfs = models.dfs::<RxyPiston, C, F>().build()?;
    println!("{dfs}");
    let dfs_processor = models.dfs::<RxyPiston, C, F>().processor()?;
    let dfs_recon = models.dfs::<RxyPiston, C, F>().reconstructor()?;

    let sh48 = models.sh48::<C>().build()?;
    println!("{sh48}");
    let sh48_processor = models.sh48::<C>().processor()?;
    println!("{:?}", sh48_processor.n_valid_lenslets());
    let sh48_recon = models.sh48::<C>().reconstructor()?;

    // let timer: Timer = Timer::new(1);
    // // <OpticalModel<_> as interface::Update>::update(&mut sh48);
    // actorscript!(
    //     1:timer[Tick]
    //         -> sh48[Frame<Dev>]!
    //             -> sh48_processor[SensorData]${48*48*3*2}
    // );
    let offaxis_om: OpticalModel<WaveSensor> =
        OpticalModelBuilder::<WaveSensorBuilder>::from(&models.dfs::<RxyPiston, C, F>().builder())
            .build()?;
    println!("{offaxis_om}");

    // let cmd =
    //     geotrans::Mirror::<geotrans::M1>::tiptilt_2_rigidbodymotions((50f64.from_mas(), 0f64));
    // // let mut cmd = vec![0f64; 42];
    // // cmd[3] = 250f64.from_mas();
    // let m1_rbm = Signals::from((cmd.clone(), 20));

    // let print = Print::default();

    // let oiwfs_opd = gif::Frame::<f64>::new("oiwfs_opd.png", 512);

    let add_m2_modes = MergeAsmCommand::new()?;
    // let t_xyz = Select::<f64>::new(0..3);
    let print = Print::default();
    let timer: Timer = Timer::new(20);
    actorscript!(
        #[model(name=ltws_oiwfs)]
        #[labels(ltws="LTWS",oiwfs="OIWFS")]
        // LTWS
        // 1: m1_rbm[M1RigidBodyMotions]
        1: timer[Tick]
            -> ltws[Frame<Dev>]!
                -> ltws_processor[SensorData]
                    -> ltws_recon[M2ASMAsmCommand]
                        -> ltws_int[M2ASMAsmCommand]
                            -> add_m2_modes[M2ASMAsmCommand]
                                -> ltws
        1: add_m2_modes[M2ASMAsmCommand] -> oiwfs
        // OIWFS
        // 1: m1_rbm[M1RigidBodyMotions]
            // -> oiwfs[Frame<Dev>]!
        1: oiwfs[Frame<Dev>]!
                -> oiwfs_processor[SensorData]
                    -> oiwfs_recon[M2GlobalTipTilt]
                        -> oiwfs_int[M2GlobalTipTilt]
                            -> add_m2_modes //oiwfs
        // 1: oiwfs_int[M2GlobalTipTilt] -> ltws
        // 1: ltws[WfeRms<-9>] -> print
        1: oiwfs[WfeRms<-9>] -> print
        1: oiwfs[SegmentWfeRms<-9>] -> print
        1: oiwfs[SegmentPiston<-9>] -> print
    );

    let dfs_m1_rbm_int = Integrator::new(42).gain(0.1);
    let dfs_m2_bm_int = Integrator::new(M2_N_MODE * 7).gain(0.1);
    // let diff_m1_rbm = Operator::new("+");
    // let add_m2_modes = Operator::new("+");
    let add_m2_modes = MergeAsmCommand::new()?;

    // let m1_rbm = Signals::from((cmd.clone(), 1600));

    let split_m12_rbm = LeftRight::<M1RbmM2modes, Split, M1RbmM2modes>::split_at(42);

    // let idx: Vec<_> = (0..7)
    // .flat_map(|i| (3..5).map(|j| i * 6 + j).collect::<Vec<_>>())
    // .collect();
    // let r_xyz = Select::<f64>::new(idx);
    // let to_mas = Fun::new(|x: &Vec<f64>| x.iter().map(|x| x.to_mas()).collect::<Vec<_>>());
    // let idx: Vec<_> = (0..7).map(|i| i * M2_N_MODE).collect();
    // let t_z = Select::<f64>::new(idx);
    // let to_nm = Fun::new(|x: &Vec<f64>| x.iter().map(|x| x * 1e9).collect::<Vec<_>>());

    let sh48_m1_bm_int = Integrator::new(M1_N_MODE * 7).gain(0.1);

    // let sh48_sampler = Sampler::default();

    // let print = Print::default();
    let dfs_print = Print::default();
    let dfs_opd = gif::Frame::<f64>::new("dfs_opd.png", 512);
    let oiwfs_opd = gif::Frame::<f64>::new("oiwfs_opd.png", 512);
    // let dfs_opd = gif::Gif::<f64>::new("dfs_opd.png", 512);
    let timer: Timer = Timer::new(200);
    actorscript!(
        #[model(name=ltws_oiwfs_dfs)]
        #[labels(ltws="LTWS",oiwfs="OIWFS",
            sh48="SH48",dfs="DFS",
            dfs_opd="DFS\nOPD",oiwfs_opd="OIWFS\nOPD")]
            // to_mas="To MAS",to_nm="To NM")]
        // LTWS
        // 1: m1_rbm[Left<M1RigidBodyMotions>]
        // -> diff_m1_rbm[M1RigidBodyMotions]
        1: timer[Tick]
            -> ltws[Frame<Dev>]!
                -> ltws_processor[SensorData]
                    -> ltws_recon[M2ASMAsmCommand]
                        -> ltws_int[M2ASMAsmCommand]
                            -> add_m2_modes[M2ASMAsmCommand]
                                -> ltws
        1: add_m2_modes[M2ASMAsmCommand] -> oiwfs
        1: add_m2_modes[M2ASMAsmCommand] -> dfs
        1: add_m2_modes[M2ASMAsmCommand] -> sh48
        // OIWFS
        // 1: diff_m1_rbm[M1RigidBodyMotions]
            // -> oiwfs[Frame<Dev>]!
        1: oiwfs[Frame<Dev>]!
                -> oiwfs_processor[M2GlobalTipTilt]
                    -> oiwfs_recon[M2GlobalTipTilt]
                        -> oiwfs_int[M2GlobalTipTilt]
                            -> add_m2_modes //oiwfs
        // SH48
        // 10: diff_m1_rbm[M1RigidBodyMotions]//${42}
            // -> sh48[Frame<Dev>]!
        10: sh48[Frame<Dev>]!
                -> sh48_processor[SensorData]//${48*48*3*2}
                    -> sh48_recon[M1ModeShapes]
                            -> sh48_m1_bm_int
        1: sh48_m1_bm_int[M1ModeShapes]//${M1_N_MODE*7}
                                -> sh48
        1: sh48_m1_bm_int[M1ModeShapes] -> dfs
        1: sh48_m1_bm_int[M1ModeShapes] -> ltws
        1: sh48_m1_bm_int[M1ModeShapes] -> oiwfs
        // DFS
        // 10: diff_m1_rbm[M1RigidBodyMotions]
        //     -> dfs[DfsFftFrame<Dev>]!
        10: dfs[DfsFftFrame<Dev>]!
                -> dfs_processor[Intercepts]//${36}
                    -> dfs_recon[M1RbmM2modes]
                        -> split_m12_rbm[M1RBM<M1RbmM2modes>]
                            -> dfs_m1_rbm_int
                                // -> diff_m1_rbm
        1: dfs_m1_rbm_int[M1RigidBodyMotions] -> dfs
        1: dfs_m1_rbm_int[M1RigidBodyMotions] -> sh48
        1: dfs_m1_rbm_int[M1RigidBodyMotions] -> ltws
        1: dfs_m1_rbm_int[M1RigidBodyMotions] -> oiwfs
        10: split_m12_rbm[M2modes<M1RbmM2modes>]
            -> dfs_m2_bm_int[SegmentPiston]
                -> add_m2_modes
        // 10: split_m12_rbm[M1RBM<M1RbmM2modes>]
        //     -> r_xyz[M1Rxy]
        //         -> to_mas[M1Rxy]
        //             -> dfs_print
        // 10: split_m12_rbm[M2modes<M1RbmM2modes>]
        //     -> t_z[M2Piston]
        //         -> to_nm[M2Piston]
        //             -> dfs_print

        // 10: ltws[WfeRms<-9>] -> dfs_print
        10: oiwfs[WfeRms<-9>] -> dfs_print
        10: oiwfs[SegmentWfeRms<-9>] -> dfs_print
        10: oiwfs[SegmentPiston<-9>] -> dfs_print

        // 20: diff_m1_rbm[M1RigidBodyMotions] -> offaxis_om
        20: dfs_m1_rbm_int[M1RigidBodyMotions] -> offaxis_om
        20: sh48_m1_bm_int[M1ModeShapes] -> offaxis_om
        20: add_m2_modes[M2ASMAsmCommand] -> offaxis_om
        // 20: oiwfs_int[M2GlobalTipTilt] -> offaxis_om
        20: offaxis_om[Wavefront] -> dfs_opd
        20: oiwfs[Wavefront] -> oiwfs_opd
    );

    oiwfs_opd.lock().await.save()?;
    dfs_opd.lock().await.save()?;

    Ok(())
}

#[derive(UID)]
pub enum M1Rxy {}
#[derive(UID)]
pub enum M2Piston {}
#[derive(UID)]
pub enum M1RbmM2modes {}
