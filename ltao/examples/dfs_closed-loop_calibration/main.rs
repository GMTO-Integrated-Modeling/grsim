//! # DFS closed-loop calibration
//!
//! A calibration script for the DFS with 2 options:
//!  1. closed-loop calibration of M1 segment tip-tilt with both
//! the LTWS and OIWFS loops closed
//! ```
//! cargo run -r --example dfs_closed-loop_calibration -- r_xyz
//! ```
//!  2. closed-loop calibration of M2 piston mode
//! ```
//! cargo run -r --example dfs_closed-loop_calibration -- piston
//! ```

use std::{env, fs::File, path::Path};

use crseo::gmt::GmtM2;
use geotrans::M2;
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Integrator, Signals};
use gmt_dos_clients_crseo::{
    calibration::{
        algebra::{Block, CalibProps, Collapse},
        Calib, Calibration, CalibrationMode, MirrorMode, Reconstructor,
    },
    DispersedFringeSensorProcessing,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, Frame, M2GlobalTipTilt, SensorData,
    },
};
use ltao::{Model, Models, M2_N_MODE};
use skyangle::Conversion;

// const N_STEP: usize = 25;

const C: usize = 1;
const F: usize = 1;

async fn closed_loop_calibration(
    sign: u32,
    s: usize,
    r_xy: usize,
    stroke: f64,
    push_pull: &mut Vec<f64>,
) -> anyhow::Result<()> {
    let models = Models::new();

    let ltws = models.ltws().build()?;
    let ltws_processor = models.ltws().processor()?;
    let ltws_recon = models.ltws().reconstructor()?;
    let ltws_int = Integrator::new(M2_N_MODE * 7).gain(0.5);

    let oiwfs = models.oiwfs().build()?;
    let oiwfs_processor = models.oiwfs().processor()?;
    let oiwfs_recon = models.oiwfs().reconstructor()?;
    let oiwfs_int = Integrator::new(2).gain(0.5);

    let dfs = models.dfs::<(), C, F>().build()?;
    let dfs_processor = models.dfs::<(), C, F>().processor()?;
    let mut cmd = vec![0f64; 42];
    cmd[r_xy + s * 6] = stroke * (-1f64).powi(sign as i32);
    let m1_rbm = Signals::from((cmd.clone(), 20));

    actorscript!(
        #[model(name=prime)]
        #[labels(ltws="LTWS",oiwfs="OIWFS")]
        // LTWS
        1: m1_rbm[M1RigidBodyMotions]
            -> ltws[Frame<Dev>]!
                -> ltws_processor[SensorData]
                    -> ltws_recon[M2ASMAsmCommand]
                        -> ltws_int[M2ASMAsmCommand]
                            -> ltws
        1: ltws_int[M2ASMAsmCommand] -> oiwfs
        // OIWFS
        1: m1_rbm[M1RigidBodyMotions]
            -> oiwfs[Frame<Dev>]!
                -> oiwfs_processor[SensorData]
                    -> oiwfs_recon[M2GlobalTipTilt]
                        -> oiwfs_int[M2GlobalTipTilt]
                            -> oiwfs
        1: oiwfs_int[M2GlobalTipTilt] -> ltws

    );
    let m1_rbm = Signals::from((cmd.clone(), 1));
    actorscript!(
        #[model(name=dfs)]
        #[labels(ltws="LTWS",oiwfs="OIWFS")]
        1: ltws_int[M2ASMAsmCommand] -> dfs
        1: oiwfs_int[M2GlobalTipTilt] -> dfs

        // DFS
        1: m1_rbm[M1RigidBodyMotions]
            -> dfs[DfsFftFrame<Dev>]
                -> dfs_processor[Intercepts]${36}
                    // -> dfs_print
    );

    let log = &mut *dfs_logging_1.lock().await;
    push_pull.extend(log.iter::<&str, f64>("Intercepts")?.flatten());
    Ok(())
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("dfs_closed-loop_calibration");
    env::set_var("DATA_REPO", &data_path);

    for arg in env::args().skip(1) {
        if &arg == "r_xy" {
            let stroke = 250f64.from_mas();
            let mut push_pull = Vec::<f64>::new();
            for sign in 0..2 {
                for s in 0..6 {
                    for r_xy in 3..5 {
                        println!("\nS{} RBM{r_xy} {:+1}\n", s + 1, (-1i32).pow(sign));
                        closed_loop_calibration(sign, s, r_xy, stroke, &mut push_pull).await?;
                    }
                }
            }

            let (push, pull) = push_pull.split_at(push_pull.len() / 2);
            let c: Vec<_> = push
                .iter()
                .zip(pull)
                .map(|(x, y)| 0.5 * (x - y) / stroke)
                .collect();

            let calib = Calib::builder()
                .sid(0)
                .c(c)
                .mask(vec![true; 36])
                .mode(
                    MirrorMode::from(CalibrationMode::r_xy(stroke))
                        .update((7, CalibrationMode::empty_rbm())),
                )
                .n_mode(6)
                .build();
            let mut recon = Reconstructor::from(calib);
            recon.pseudoinverse();
            println!("{recon}");
            serde_pickle::to_writer(
                &mut File::create(data_path.join("calib_ltws-oiwfs_dfs_m1-rxy.pkl"))?,
                &recon,
                Default::default(),
            )?;
        }
        if &arg == "piston" {
            let stroke = 250e-9;
            let models = Models::new();
            let mut recon = <DispersedFringeSensorProcessing as Calibration<GmtM2>>::calibrate(
                &models.dfs::<(), 1, 1>().builder(),
                MirrorMode::from(
                    CalibrationMode::modes(M2_N_MODE, stroke)
                        .start_from(1)
                        .ends_at(1),
                )
                .update((7, CalibrationMode::empty_modes(M2_N_MODE))),
            )?;
            recon.pseudoinverse();
            println!("{recon}");
            serde_pickle::to_writer(
                &mut File::create(data_path.join("calib_dfs_m2-piston.pkl"))?,
                &recon,
                Default::default(),
            )?;
        }
        if &arg == "merged" {
            let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("examples")
                .join("dfs_closed-loop_calibration");
            // .join("calib_ltws-oiwfs_dfs_m1-rxy.pkl");
            // println!(" -- DFS CALIBRATION -- ");
            // Ok(serde_pickle::from_reader(
            //     &mut File::open(data_path)?,
            //     Default::default(),
            // )?)
            let recon_rxy: Reconstructor<MirrorMode> = serde_pickle::from_reader(
                &mut File::open(data_path.join("calib_ltws-oiwfs_dfs_m1-rxy.pkl"))?,
                Default::default(),
            )?;
            let recon_tz: Reconstructor<CalibrationMode> = serde_pickle::from_reader(
                &mut File::open(data_path.join("calib_dfs_m2-piston.pkl"))?,
                Default::default(),
            )?;
            let recon_tz = recon_tz.collapse();
            // let mut split_recon_rxy = recon_rxy.split();
            // split_recon_rxy.merge(recon_tz);
            // let mut recon = split_recon_rxy.collapse();
            let mut recon = Block::block(&[&[&recon_rxy, &recon_tz]]);
            // recon.pseudoinverse();
            recon.truncated_pseudoinverse(vec![2]);
            println!("{recon}");
            let svd = recon.calib_slice()[0].mat_ref().svd();
            println!("Singular values:\n{:#?}", svd.s_diagonal());
        }
    }

    Ok(())
}
