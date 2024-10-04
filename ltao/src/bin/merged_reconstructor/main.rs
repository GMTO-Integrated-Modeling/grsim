use std::{fs::File, path::Path};

use crseo::{imaging::LensletArray, FromBuilder, Gmt, Source};
use faer::{mat::from_column_major_slice, solvers::SolverCore};
use gmt_dos_clients::print;
use gmt_dos_clients_crseo::{
    calibration::{
        Block, Calib, CalibProps, CalibrationMode, ClosedLoopCalib, ClosedLoopCalibrate, Collapse,
        MirrorMode, Reconstructor,
    },
    sensors::{
        Camera, DispersedFringeSensor, DispersedFringeSensorProcessing, NoSensor, WaveSensor,
    },
    Centroids, DeviceInitialize, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m1::{M1ModeShapes, M1RigidBodyMotions},
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, Frame, SensorData, WfeRms,
    },
};
use grsim_ltao::{MergedReconstructor, Sh48Data};
use interface::{Read, Update, Write};
use skyangle::Conversion;

const M1_N_MODE: usize = 27;
const M2_N_MODE: usize = 66;
const AGWS_N_GS: usize = 3;

type DFS = DispersedFringeSensor<1, 1>;

fn main() -> anyhow::Result<()> {
    let path = Path::new("src/bin/merged_reconstructor");
    let gmt_builder = Gmt::builder()
        .m1("bending modes", M1_N_MODE)
        .m2("Karhunen-Loeve", M2_N_MODE)
        .m1_truss_projection(false);

    let agws_gs_builder = Source::builder()
        .size(AGWS_N_GS)
        .on_ring(6f32.from_arcmin());

    let sh48 = Camera::builder()
        .n_sensor(AGWS_N_GS)
        .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32))
        .lenslet_flux(0.75);

    let sh48_om_builder = OpticalModel::<Camera<1>>::builder()
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone())
        .sensor(sh48.clone());

    let dfs_om_builder = OpticalModel::<DFS>::builder()
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone())
        .sensor(
            DFS::builder()
                .source(agws_gs_builder.clone())
                .nyquist_factor(3.),
        );

    let m1_rbm = CalibrationMode::r_xy(0.1f64.from_arcsec());
    let m1_bm = CalibrationMode::modes(M1_N_MODE, 1e-4);
    let m2_mode = CalibrationMode::modes(M2_N_MODE, 1e-6);

    // SH48 BMS CALIBRATION
    let mut calib_sh48_bm_bd: Reconstructor<MirrorMode> =
        if let Ok(file) = File::open(path.join("calib_sh48_bm.pkl")) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let closed_loop_optical_model =
                OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone());
            let mut calib_sh48_bm = <Centroids as ClosedLoopCalibrate<WaveSensor>>::calibrate(
                sh48_om_builder.clone().into(),
                m1_bm.clone(),
                closed_loop_optical_model,
                m2_mode.clone().start_from(2),
            )?;
            calib_sh48_bm.pseudoinverse();
            println!("{calib_sh48_bm}");
            let calib_sh48_bm_bd = calib_sh48_bm.diagonal();
            serde_pickle::to_writer(
                &mut File::create(path.join("calib_sh48_bm.pkl"))?,
                &calib_sh48_bm_bd,
                Default::default(),
            )?;
            calib_sh48_bm_bd
        };
    println!("SH48 BMS CALIBRATION: {}", calib_sh48_bm_bd.pseudoinverse());

    // SH48 RBMS CALIBRATION
    let mut calib_sh48_rbm_bd: Reconstructor<MirrorMode> =
        if let Ok(file) = File::open(path.join("calib_sh48_rbm.pkl")) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let closed_loop_optical_model =
                OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone());
            let mut calib_sh48_rbm = <Centroids as ClosedLoopCalibrate<WaveSensor>>::calibrate(
                sh48_om_builder.clone().into(),
                m1_rbm.clone(),
                closed_loop_optical_model,
                m2_mode.clone().start_from(2),
            )?;
            calib_sh48_rbm.pseudoinverse();
            println!("{calib_sh48_rbm}");
            let calib_sh48_rbm_bd = calib_sh48_rbm.diagonal();
            serde_pickle::to_writer(
                &mut File::create(path.join("calib_sh48_rbm.pkl"))?,
                &calib_sh48_rbm_bd,
                Default::default(),
            )?;
            calib_sh48_rbm_bd
        };
    println!(
        "SH48 RBMS CALIBRATION: {}",
        calib_sh48_rbm_bd.pseudoinverse()
    );

    // DFS RBMS CALIBRATION
    let mut calib_dfs_rbm_col: Reconstructor<MirrorMode> =
        if let Ok(file) = File::open(path.join("calib_dfs_rbm.pkl")) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let mut calib_dfs_rbm =
                <DispersedFringeSensorProcessing as ClosedLoopCalibrate<WaveSensor>>::calibrate(
                    dfs_om_builder.clone(),
                    m1_rbm.clone(),
                    OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
                    m2_mode.clone(),
                )?;
            calib_dfs_rbm.pseudoinverse();
            let calib_dfs_rbm_col = calib_dfs_rbm.collapse();
            serde_pickle::to_writer(
                &mut File::create(path.join("calib_dfs_rbm.pkl"))?,
                &calib_dfs_rbm_col,
                Default::default(),
            )?;
            calib_dfs_rbm_col
        };
    println!(
        "DFS RBMS CALIBRATION: {}",
        calib_dfs_rbm_col.pseudoinverse()
    );

    // DFS BMS CALIBRATION
    let mut calib_dfs_bm_col: Reconstructor<MirrorMode> =
        if let Ok(file) = File::open(path.join("calib_dfs_bm.pkl")) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let mut calib_dfs_bm =
                <DispersedFringeSensorProcessing as ClosedLoopCalibrate<WaveSensor>>::calibrate(
                    dfs_om_builder.clone(),
                    m1_bm.clone(),
                    OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
                    m2_mode.clone(),
                )?;
            calib_dfs_bm.pseudoinverse();
            let calib_dfs_bm_col = calib_dfs_bm.collapse();
            serde_pickle::to_writer(
                &mut File::create(path.join("calib_dfs_bm.pkl"))?,
                &calib_dfs_bm_col,
                Default::default(),
            )?;
            calib_dfs_bm_col
        };
    println!("DFS BMS CALIBRATION: {}", calib_dfs_bm_col.pseudoinverse());

    // calib_sh48_rbm_bd.pseudoinverse();
    // println!("{calib_sh48_rbm_bd}");
    // calib_sh48_bm_bd.pseudoinverse();
    // println!("{calib_sh48_bm_bd}");

    println!("Merging the SH48 RBMs & BMs calibration matrices");
    let mut sh48_merged_recon =
        <Reconstructor<_> as Block>::block(&[&[&calib_sh48_rbm_bd, &calib_sh48_bm_bd]]);
    sh48_merged_recon.normalize();
    sh48_merged_recon.pseudoinverse();
    println!("{sh48_merged_recon}");

    println!("Normalizing & merging the SH48 RBMs & BMs calibration matrices");
    let mut sh48_merged_recon0 =
        <Reconstructor<_> as Block>::block(&[&[&calib_sh48_rbm_bd, &calib_sh48_bm_bd]]);
    let norm_sh48_rbm = calib_sh48_rbm_bd.normalize();
    let norm_sh48_bm = calib_sh48_bm_bd.normalize();
    let mut sh48_merged_recon =
        <Reconstructor<_> as Block>::block(&[&[&calib_sh48_rbm_bd, &calib_sh48_bm_bd]]);
    let norm_sh48 = sh48_merged_recon.normalize();
    sh48_merged_recon.pseudoinverse();
    println!("{sh48_merged_recon}");

    let calib_dfs_rbm_7: Reconstructor<_> = Calib::<MirrorMode>::builder()
        .sid(7)
        .c(vec![0.; 36 * 2])
        .n_mode(6)
        .n_cols(2)
        .mode(m1_rbm.into())
        .mask(vec![true; 36])
        .build()
        .into();

    let calib_dfs_bm_7: Reconstructor<_> = Calib::<MirrorMode>::builder()
        .sid(7)
        .c(vec![0.; 36 * M1_N_MODE])
        .n_mode(M1_N_MODE)
        .n_cols(M1_N_MODE)
        .mode(m1_bm.into())
        .mask(vec![true; 36])
        .build()
        .into();

    println!("Merging the DFS RBMs & BMs calibration matrices");
    let mut dfs_merged_recon = <Reconstructor<_> as Block>::block(&[&[
        &calib_dfs_rbm_col,
        &calib_dfs_rbm_7,
        &calib_dfs_bm_col,
        &calib_dfs_bm_7,
    ]]);
    dfs_merged_recon.normalize();
    dfs_merged_recon.pseudoinverse();
    println!("{dfs_merged_recon}");

    println!("Normalizing & merging the DFS RBMs & BMs calibration matrices");
    let mut dfs_merged_recon0 = <Reconstructor<_> as Block>::block(&[&[
        &calib_dfs_rbm_col,
        &calib_dfs_rbm_7,
        &calib_dfs_bm_col,
        &calib_dfs_bm_7,
    ]]);
    let norm_dfs_rbm = calib_dfs_rbm_col.normalize();
    let norm_dfs_bm = calib_dfs_bm_col.normalize();
    let mut dfs_merged_recon = <Reconstructor<_> as Block>::block(&[&[
        &calib_dfs_rbm_col,
        &calib_dfs_rbm_7,
        &calib_dfs_bm_col,
        &calib_dfs_bm_7,
    ]]);
    let norm_dfs = dfs_merged_recon.normalize();
    dfs_merged_recon.pseudoinverse();
    println!("{dfs_merged_recon}");

    println!("Merged reconstructor");
    let mut merged_recon0 =
        <Reconstructor<_> as Block>::block(&[&[&sh48_merged_recon0], &[&dfs_merged_recon0]]);
    let mut q = sh48_merged_recon0.clone();
    q.normalize();
    let mut merged_recon = <Reconstructor<_> as Block>::block(&[&[&q], &[&dfs_merged_recon]]);
    println!("{merged_recon}");
    println!("Computing the pseudo-inverse of the merged reconstructor ...");
    let now = std::time::Instant::now();
    merged_recon.pseudoinverse();
    println!("... done in {:?}", now.elapsed());
    println!("{merged_recon}");

    let a_mat = merged_recon.pinv().next().unwrap() * merged_recon0.calib().next().unwrap();
    println!("{:?}", a_mat.shape());

    let lu = a_mat.partial_piv_lu();
    let inv = lu.inverse();
    println!("{:?}", inv.shape());

    // TESTING

    let mut dfs_processor = DispersedFringeSensorProcessing::new();
    dfs_om_builder.initialize(&mut dfs_processor);

    let mut sh48_centroids: Centroids = Centroids::try_from(&sh48)?;
    sh48_om_builder.initialize(&mut sh48_centroids);
    dbg!(sh48_centroids.n_valid_lenslets());

    let mut dfs_om = dfs_om_builder.build()?;
    println!("{dfs_om}");

    let mut sh48_om = sh48_om_builder.build()?;
    println!("{sh48_om}");

    let dfs_recon: Reconstructor<CalibrationMode, ClosedLoopCalib> = serde_pickle::from_reader(
        File::open("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-rxy_v2.pkl")?,
        Default::default(),
    )?;
    let mut sh48_recon: Reconstructor<CalibrationMode, ClosedLoopCalib> =
        serde_pickle::from_reader(
            File::open("src/bin/agws_oiwfs/calib_sh48_bm.pkl")?,
            Default::default(),
        )?;

    let mut m1_rxy = vec![vec![0f64; 2]; 7];
    m1_rxy[0][0] = 0.1f64.from_arcsec();
    m1_rxy[1][1] = 0.1f64.from_arcsec();
    let m1_rxy_cmd: Vec<_> = dfs_recon
        .calib_slice()
        .iter()
        .zip(&m1_rxy)
        .map(|(c, m1_rxy)| c.m1_to_m2() * -faer::mat::from_column_major_slice::<f64>(m1_rxy, 2, 1))
        .flat_map(|m| m.col_as_slice(0).to_vec())
        .chain(vec![0.; M2_N_MODE])
        .collect();

    let mut m1_bm = vec![vec![0f64; M1_N_MODE]; 7];
    // m1_bm[0][0] = 1e-4;
    // m1_bm[6][4] = 1e-5;
    let m1_bm_cmd: Vec<_> = sh48_recon
        .calib_slice()
        .iter()
        .zip(&m1_bm)
        .map(|(c, m1_bm)| {
            c.m1_to_m2() * -faer::mat::from_column_major_slice::<f64>(&m1_bm, M1_N_MODE, 1)
        })
        .flat_map(|m2_fit| {
            let mut cmd = vec![0.];
            cmd.extend(m2_fit.col_as_slice(0));
            cmd
        })
        .collect();
    let cmd: Vec<_> = m1_rxy_cmd
        .into_iter()
        .zip(m1_bm_cmd.into_iter())
        .map(|(x, y)| x + y)
        .collect();
    println!("cmds: {:?}", cmd.len());

    let mut m1_rbm: Vec<f64> = m1_rxy
        .into_iter()
        .flat_map(|rxy| {
            vec![0.; 3]
                .into_iter()
                .chain(rxy.into_iter())
                .chain(Some(0.))
                .collect::<Vec<_>>()
        })
        .collect();

    let mut om = OpticalModel::<NoSensor>::builder()
        .gmt(gmt_builder.clone())
        .build()?;
    // dbg!(&cmd[..10]);
    <OpticalModel<NoSensor> as Read<M1RigidBodyMotions>>::read(&mut om, m1_rbm.clone().into());
    <OpticalModel<NoSensor> as Read<M1ModeShapes>>::read(
        &mut om,
        m1_bm.iter().cloned().flatten().collect::<Vec<_>>().into(),
    );
    <OpticalModel<NoSensor> as Read<M2ASMAsmCommand>>::read(&mut om, cmd.clone().into());
    om.update();
    dbg!(<OpticalModel as Write<WfeRms<-9>>>::write(&mut om));

    // DFS
    <OpticalModel<DFS> as Read<M1RigidBodyMotions>>::read(&mut dfs_om, m1_rbm.clone().into());
    <OpticalModel<DFS> as Read<M1ModeShapes>>::read(
        &mut dfs_om,
        m1_bm.iter().cloned().flatten().collect::<Vec<_>>().into(),
    );
    <OpticalModel<DFS> as Read<M2ASMAsmCommand>>::read(&mut dfs_om, cmd.clone().into());

    dfs_om.update();

    <OpticalModel<DFS> as Write<DfsFftFrame<Dev>>>::write(&mut dfs_om).map(|data| {
        <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(&mut dfs_processor, data)
    });
    dfs_processor.update();
    let dfs_y =
        <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor).unwrap();
    dbg!(dfs_y.len());
    let mut dfs_recon = dfs_recon.collapse();
    dfs_recon.pseudoinverse();
    println!("{dfs_recon}");

    // let q = calib_dfs_rbm_col.normalize();
    calib_dfs_rbm_col.pseudoinverse();
    println!("{calib_dfs_rbm_col}");

    dfs_merged_recon0.pseudoinverse();
    println!("{dfs_merged_recon0}");

    let tt = faer::mat::from_column_major_slice::<f64>(&dfs_y.as_arc(), 36, 1) / &dfs_merged_recon0;

    tt[0]
        .col_as_slice(0)
        .chunks(2)
        .take(7)
        .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .for_each(|x| println!("{:+5.0?}", x));

    // SH48
    <OpticalModel<Camera<1>> as Read<M1RigidBodyMotions>>::read(
        &mut sh48_om,
        m1_rbm.clone().into(),
    );
    <OpticalModel<Camera<1>> as Read<M1ModeShapes>>::read(
        &mut sh48_om,
        m1_bm.iter().cloned().flatten().collect::<Vec<_>>().into(),
    );
    <OpticalModel<Camera<1>> as Read<M2ASMAsmCommand>>::read(&mut sh48_om, cmd.into());

    sh48_om.update();

    <OpticalModel<Camera<1>> as Write<Frame<Dev>>>::write(&mut sh48_om)
        .map(|data| <Centroids as Read<Frame<Dev>>>::read(&mut sh48_centroids, data));
    sh48_centroids.update();
    let sh48_y0 = <Centroids as Write<Sh48Data>>::write(&mut sh48_centroids).unwrap();
    let sh48_y = sh48_merged_recon0.calib_slice()[0].mask(&sh48_y0.as_arc());

    dbg!(sh48_y.len());

    // println!("{sh48_recon}");
    calib_sh48_bm_bd.pseudoinverse();
    // println!("{calib_sh48_bm_bd}");e

    // let mut sh48_recon_bd = sh48_recon.diagonal();
    // sh48_recon_bd.pseudoinverse();
    // println!("{sh48_recon_bd}");
    // serde_pickle::to_writer(
    //     &mut File::create(path.join("sh48_recon_bd.pkl"))?,
    //     &sh48_recon_bd,
    //     Default::default(),
    // )?;

    serde_pickle::to_writer(
        &mut File::create(path.join("sh48_dfs_y.pkl"))?,
        &(&sh48_y, &dfs_y),
        Default::default(),
    )?;

    // let bm = sh48_merged_recon0
    //     .calib_pinv()
    //     .next()
    //     .map(|(c, ic)| {
    //         let y = c.mask(&sh48_y);
    //         ic * faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1)
    //     })
    //     .unwrap();
    sh48_merged_recon0.pseudoinverse();
    let bm =
        faer::mat::from_column_major_slice::<f64>(&sh48_y, sh48_y.len(), 1) / &sh48_merged_recon0;
    dbg!(bm[0].shape());
    bm[0].col_as_slice(0)[14..].chunks(M1_N_MODE).for_each(|x| {
        println!(
            "{:+7.3?}",
            x.iter().take(6).map(|x| x * 1e4).collect::<Vec<_>>()
        )
    });

    let y: Vec<_> = sh48_y
        .into_iter()
        .chain(dfs_y.as_arc().iter().cloned())
        .collect();
    dbg!(y.len());

    merged_recon0.pseudoinverse();
    println!("{merged_recon0}");

    merged_recon
        .pinv()
        .next()
        .map(|pinv| pinv.transform(|mat: faer::MatRef<'_, f64>| &inv * mat));
    let c = faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1) / &merged_recon;
    // let ac = inv * &c[0];
    c[0].col_as_slice(0)
        .chunks(2)
        .take(7)
        .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .for_each(|x| println!("{:+5.0?}", x));
    c[0].col_as_slice(0)[14..].chunks(M1_N_MODE).for_each(|x| {
        println!(
            "{:+7.3?}",
            x.iter().take(6).map(|x| x * 1e4).collect::<Vec<_>>()
        )
    });

    serde_pickle::to_writer(
        &mut File::create(path.join("merged-reconstructor.pkl"))?,
        &(
            &merged_recon,
            sh48_merged_recon0.calib_slice()[0].mask_slice(),
        ),
        Default::default(),
    )?;

    let mut merged = MergedReconstructor::new();
    <MergedReconstructor as Read<Intercepts>>::read(&mut merged, dfs_y);
    <MergedReconstructor as Read<Sh48Data>>::read(&mut merged, sh48_y0);
    merged.update();
    let rbm = <MergedReconstructor as Write<M1RigidBodyMotions>>::write(&mut merged).unwrap();
    let bm = <MergedReconstructor as Write<M1ModeShapes>>::write(&mut merged).unwrap();
    rbm.chunks(6)
        .map(|rbm| {
            rbm.iter()
                .skip(3)
                .take(2)
                .map(|x| x.to_mas())
                .collect::<Vec<_>>()
        })
        .for_each(|x| println!("{:+5.0?}", x));
    bm.chunks(M1_N_MODE).for_each(|x| {
        println!(
            "{:+7.3?}",
            x.iter().take(6).map(|x| x * 1e4).collect::<Vec<_>>()
        )
    });

    Ok(())
}
