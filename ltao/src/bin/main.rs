use std::{env, path::Path};

use crseo::{Atmosphere, FromBuilder, RayTracing};
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
    optics::{M2GlobalTipTilt, SegmentPiston, SegmentWfeRms, Wavefront, WfeRms},
};
use interface::{Data, Read, Tick, Update, Write, UID};
use ltao::{
    kernels::KernelFrame, Dfs, Ltws, M1RbmM2modes, MergeAsmCommand, Model, Models, Oiwfs,
    RxyPiston, Sh48, M1_N_MODE, M2_N_MODE,
};

// const N_STEP: usize = 25;

const DFS_CAM_INT: usize = 5;
const DFS_FFT_INT: usize = 500;
const SH48_INT: usize = 2500;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("bin");
    env::set_var("DATA_REPO", &data_path);

    let sampling_frequency = 500f64;
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

    let models = Models::new().atmosphere(sampling_frequency, atm_builder);

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
        1: timer[Tick]
            -> ltws[KernelFrame<Ltws>]!
                // -> ltws_processor[SensorData]
                    // -> ltws_recon[M2ASMAsmCommand]
                        -> ltws_kernel[M2ASMAsmCommand]
                            -> add_m2_modes[M2ASMAsmCommand]
                                -> ltws
        1: add_m2_modes[M2ASMAsmCommand] -> oiwfs
        // OIWFS
        // 1: m1_rbm[M1RigidBodyMotions]
            // -> oiwfs[Frame<Dev>]!
        1: oiwfs[KernelFrame<Oiwfs>]!
                // -> oiwfs_processor[SensorData]
                    // -> oiwfs_recon[M2GlobalTipTilt]
                        -> oiwfs_kernel[M2GlobalTipTilt]
                            -> add_m2_modes //oiwfs
        // 1: oiwfs_int[M2GlobalTipTilt] -> ltws
        // 1: ltws[WfeRms<-9>] -> print
        1: oiwfs[WfeRms<-9>]$ -> print
        1: oiwfs[SegmentWfeRms<-9>]$ -> print
        1: oiwfs[SegmentPiston<-9>]$ -> print
        1: oiwfs[Wavefront]$ -> oiwfs0_opd
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
    let timer: Timer = Timer::new(SH48_INT * 10);
    actorscript!(
        #[model(name=ltws_oiwfs_dfs)]
        #[labels(ltws="GMT w/\n🌫  & LTWS",oiwfs="GMT w/\n🌫  & OIWFS",
            sh48="GMT w/\n🌫  & SH48",dfs="GMT w/\n🌫  & DFS")]
            // to_mas="To MAS",to_nm="To NM")]
        // LTWS
        // 1: m1_rbm[Left<M1RigidBodyMotions>]
        // -> diff_m1_rbm[M1RigidBodyMotions]
        1: timer[Tick]
            -> ltws[KernelFrame<Ltws>]!
                // -> ltws_processor[SensorData]
                    // -> ltws_recon[M2ASMAsmCommand]
                        -> ltws_kernel[M2ASMAsmCommand]
                            -> add_m2_modes[M2ASMAsmCommand]
                                -> ltws
        1: add_m2_modes[M2ASMAsmCommand] -> oiwfs
        1: add_m2_modes[M2ASMAsmCommand] -> dfs
        1: add_m2_modes[M2ASMAsmCommand] -> sh48
        // OIWFS
        // 1: diff_m1_rbm[M1RigidBodyMotions]
            // -> oiwfs[Frame<Dev>]!
        1: oiwfs[KernelFrame<Oiwfs>]!
                // -> oiwfs_processor[M2GlobalTipTilt]
                    // -> oiwfs_recon[M2GlobalTipTilt]
                        -> oiwfs_kernel[M2GlobalTipTilt]
                            -> add_m2_modes //oiwfs
        // SH48
        // 10: diff_m1_rbm[M1RigidBodyMotions]//${42}
            // -> sh48[Frame<Dev>]!
        2500: sh48[KernelFrame<Sh48<SH48_INT>>]!
                // -> sh48_processor[SensorData]//${48*48*3*2}
                    // -> sh48_recon[M1ModeShapes]
                            -> sh48_kernel
        // 1: sh48[Frame<Host>] -> sh48_frame
        1: sh48_kernel[M1ModeShapes]//${M1_N_MODE*7}
                                -> sh48
        1: sh48_kernel[M1ModeShapes] -> dfs
        1: sh48_kernel[M1ModeShapes] -> ltws
        1: sh48_kernel[M1ModeShapes] -> oiwfs
        // DFS
        // 10: diff_m1_rbm[M1RigidBodyMotions]
        //     -> dfs[DfsFftFrame<Dev>]!
        2500: dfs[KernelFrame<Dfs<RxyPiston,DFS_CAM_INT,DFS_FFT_INT>>]!
                // -> dfs_processor[Intercepts]//${36}
                    // -> dfs_recon[M1RbmM2modes]
                            -> dfs_kernel[M1RbmM2modes]
                        -> split_m12_rbm//[M1RBM<M1RbmM2modes>]
                                // -> diff_m1_rbm
        1: split_m12_rbm[M1RigidBodyMotions] -> dfs
        1: split_m12_rbm[M1RigidBodyMotions] -> sh48
        1: split_m12_rbm[M1RigidBodyMotions] -> ltws
        1: split_m12_rbm[M1RigidBodyMotions] -> oiwfs
        // 2500: split_m12_rbm[M2modes<M1RbmM2modes>]
            // -> dfs_m2_bm_int
        1: split_m12_rbm[SegmentPiston]
                -> add_m2_modes
        // 2500: split_m12_rbm[M1RBM<M1RbmM2modes>]
        //     -> r_xyz[M1Rxy]
        //         -> to_mas[M1Rxy]${14}
        //             // -> dfs_print
        // 2500: split_m12_rbm[M2modes<M1RbmM2modes>]
        //     // -> t_z[M2Piston]
        //         -> to_nm[M2Piston]${7}
        //             // -> dfs_print

        // 10: ltws[WfeRms<-9>] -> dfs_print
        1: oiwfs[WfeRms<-9>]$
        1: oiwfs[SegmentWfeRms<-9>]$
        1: oiwfs[SegmentPiston<-9>]$
        10: oiwfs[WfeRms<-9>] -> dfs_print
        10: oiwfs[SegmentWfeRms<-9>] -> dfs_print
        10: oiwfs[SegmentPiston<-9>] -> dfs_print

        // 20: diff_m1_rbm[M1RigidBodyMotions] -> offaxis_om
        // 20: dfs_m1_rbm_int[M1RigidBodyMotions] -> offaxis_om
        // 20: sh48_m1_bm_int[M1ModeShapes] -> offaxis_om
        // 20: add_m2_modes[M2ASMAsmCommand] -> offaxis_om
        // 20: oiwfs_int[M2GlobalTipTilt] -> offaxis_om
        // 20: offaxis_om[Wavefront] -> dfs_opd
        // 1-1: oiwfs[Wavefront]$// -> oiwfs_opd
        1: split_m12_rbm[M1RigidBodyMotions]${42}
        1: split_m12_rbm[SegmentPiston]${7}
        1: sh48_kernel[M1ModeShapes]${M1_N_MODE*7}
        1: add_m2_modes[M2ASMAsmCommand]${M2_N_MODE *7}
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
        <LeftRight<M1RbmM2modes, Split, M1RbmM2modes> as Write<M1RBM<M1RbmM2modes>>>::write(
            &mut self.0,
        )
        .map(|data| data.transmute::<M1RigidBodyMotions>())
    }
}
impl Write<SegmentPiston> for DfsSplit {
    fn write(&mut self) -> Option<Data<SegmentPiston>> {
        <LeftRight<M1RbmM2modes, Split, M1RbmM2modes> as Write<M2modes<M1RbmM2modes>>>::write(
            &mut self.0,
        )
        .map(|data| data.transmute::<SegmentPiston>())
    }
}
