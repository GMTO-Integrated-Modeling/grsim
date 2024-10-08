use std::{fs::File, path::Path};

use crseo::{gmt::GmtM1, imaging::LensletArray, FromBuilder, Gmt, Source};
use faer::mat::from_column_major_slice;
use gmt_dos_clients_crseo::{
    calibration::{
        algebra::{Block, CalibProps, Collapse},
        Calib, Calibrate, CalibrationMode, ClosedLoopCalib, ClosedLoopCalibrate,
        ClosedLoopReconstructor, MirrorMode, MixedMirrorMode, Reconstructor,
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
        Dev, Frame, WfeRms,
    },
};
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

    // let m1_rbm = CalibrationMode::r_xy(0.1f64.from_arcsec());
    let mut m1_rbm = [CalibrationMode::RBM([
        Some(1e-6),
        Some(1e-6),
        None,
        Some(100f64.from_mas()),
        Some(100f64.from_mas()),
        Some(100f64.from_mas()),
    ]); 7];
    m1_rbm[6] = CalibrationMode::RBM([
        Some(1e-6),
        Some(1e-6),
        None,
        Some(100f64.from_mas()),
        Some(100f64.from_mas()),
        None,
    ]);
    let m1_bm = CalibrationMode::modes(M1_N_MODE, 1e-4);
    let m2_mode = CalibrationMode::modes(M2_N_MODE, 1e-6);
    let m1_tz = MirrorMode::from(CalibrationMode::t_z(1e-6)).remove(7);

    // SH48 BMS CALIBRATION
    let calib_sh48_bm: ClosedLoopReconstructor =
        if let Ok(file) = File::open(path.join("calib_sh48_bm.pkl")) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let closed_loop_optical_model =
                OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone());
            let mut calib_sh48_bm = <Centroids as ClosedLoopCalibrate<WaveSensor>>::calibrate(
                &((&sh48_om_builder).into()),
                m1_bm.clone(),
                &closed_loop_optical_model,
                m2_mode.clone().start_from(2),
            )?;
            calib_sh48_bm.pseudoinverse();
            println!("{calib_sh48_bm}");
            // let calib_sh48_bm_bd = calib_sh48_bm.diagonal();
            serde_pickle::to_writer(
                &mut File::create(path.join("calib_sh48_bm.pkl"))?,
                &calib_sh48_bm,
                Default::default(),
            )?;
            calib_sh48_bm
        };
    println!("SH48 BMS CALIBRATION: {}", calib_sh48_bm);

    // SH48 RBMS CALIBRATION
    let mut calib_sh48_rbm: ClosedLoopReconstructor =
        if let Ok(file) = File::open(path.join("calib_sh48_rbm.pkl")) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let closed_loop_optical_model =
                OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone());
            let mut calib_sh48_rbm = <Centroids as ClosedLoopCalibrate<WaveSensor>>::calibrate(
                &((&sh48_om_builder).into()),
                m1_rbm.clone(),
                &closed_loop_optical_model,
                m2_mode.clone().start_from(2),
            )?;
            calib_sh48_rbm.pseudoinverse();
            println!("{calib_sh48_rbm}");
            // let calib_sh48_rbm_bd = calib_sh48_rbm.diagonal();
            serde_pickle::to_writer(
                &mut File::create(path.join("calib_sh48_rbm.pkl"))?,
                &calib_sh48_rbm,
                Default::default(),
            )?;
            calib_sh48_rbm
        };
    println!("SH48 RBMS CALIBRATION: {}", calib_sh48_rbm);

    // SH48 Tz CALIBRATION
    let calib_sh48_tz: Reconstructor = if let Ok(file) = File::open(path.join("calib_sh48_tz.pkl"))
    {
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let mut calib_sh48_tz = <Centroids as Calibrate<GmtM1>>::calibrate(
            &((&sh48_om_builder).into()),
            m1_tz.clone(),
        )?;
        calib_sh48_tz.pseudoinverse();
        println!("{calib_sh48_tz}");
        // let calib_sh48_tz_bd = calib_sh48_tz.diagonal();
        serde_pickle::to_writer(
            &mut File::create(path.join("calib_sh48_tz.pkl"))?,
            &calib_sh48_tz,
            Default::default(),
        )?;
        calib_sh48_tz
    };
    println!("SH48 Tz CALIBRATION: {}", calib_sh48_tz);

    // DFS RBMS CALIBRATION
    let mut calib_dfs_rbm: ClosedLoopReconstructor = if let Ok(file) =
        File::open(path.join("calib_dfs_rbm.pkl"))
    {
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let mut calib_dfs_rbm =
            <DispersedFringeSensorProcessing as ClosedLoopCalibrate<WaveSensor>>::calibrate_serial(
                &dfs_om_builder,
                m1_rbm,
                &OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
                m2_mode.clone(),
            )?;
        calib_dfs_rbm.pseudoinverse();
        // let calib_dfs_rbm_col = calib_dfs_rbm.collapse();
        serde_pickle::to_writer(
            &mut File::create(path.join("calib_dfs_rbm.pkl"))?,
            &calib_dfs_rbm,
            Default::default(),
        )?;
        calib_dfs_rbm
    };
    println!("DFS RBMS CALIBRATION: {}", calib_dfs_rbm);

    // DFS Tz CALIBRATION
    let calib_dfs_tz: Reconstructor = if let Ok(file) = File::open(path.join("calib_dfs_tz.pkl")) {
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let mut calib_dfs_tz =
            <DispersedFringeSensorProcessing as Calibrate<GmtM1>>::calibrate_serial(
                &dfs_om_builder,
                m1_tz,
            )?;
        calib_dfs_tz.pseudoinverse();
        // let calib_dfs_tz_col = calib_dfs_tz.collapse();
        serde_pickle::to_writer(
            &mut File::create(path.join("calib_dfs_tz.pkl"))?,
            &calib_dfs_tz,
            Default::default(),
        )?;
        calib_dfs_tz
    };
    println!("DFS Tz CALIBRATION: {}", calib_dfs_tz);

    // DFS BMS CALIBRATION
    let calib_dfs_bm: ClosedLoopReconstructor = if let Ok(file) =
        File::open(path.join("calib_dfs_bm.pkl"))
    {
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let mut calib_dfs_bm =
            <DispersedFringeSensorProcessing as ClosedLoopCalibrate<WaveSensor>>::calibrate_serial(
                &dfs_om_builder,
                m1_bm,
                &OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
                m2_mode.clone(),
            )?;
        calib_dfs_bm.pseudoinverse();
        // let calib_dfs_bm_col = calib_dfs_bm.collapse();
        serde_pickle::to_writer(
            &mut File::create(path.join("calib_dfs_bm.pkl"))?,
            &calib_dfs_bm,
            Default::default(),
        )?;
        calib_dfs_bm
    };
    println!("DFS BMS CALIBRATION: {}", calib_dfs_bm);

    println!("==============================================");

    let calib_sh48_bm_bd = calib_sh48_bm.diagonal();
    println!("SH48 BMS block-diagonal: {calib_sh48_bm_bd}");
    // println!("{calib_sh48_rbm}");
    // println!("{calib_sh48_tz}");

    // println!("{calib_sh48_rbm}");
    let calib_sh48_rbm_bd = calib_sh48_rbm.merge(calib_sh48_tz).diagonal();
    println!("SH48 RBMS block-diagonal: {calib_sh48_rbm_bd}");

    let calib_dfs_bm_col = calib_dfs_bm.collapse();
    println!("DFS BMS block-diagonal: {calib_dfs_bm_col}");
    let calib_dfs_rbm_col = calib_dfs_rbm.merge(calib_dfs_tz).clone().collapse();
    println!("DFS RBMS block-diagonal: {calib_dfs_rbm_col}");

    let mut sh48_merged_recon =
        <Reconstructor<MirrorMode> as Block>::block(&[&[&calib_sh48_rbm_bd, &calib_sh48_bm_bd]]);
    println!("SH48 merged: {sh48_merged_recon}");

    let mut dfs_merged_recon =
        <Reconstructor<MirrorMode> as Block>::block(&[&[&calib_dfs_rbm_col, &calib_dfs_bm_col]]);
    println!("DFS merged: {dfs_merged_recon}");

    let norm_sh48 = sh48_merged_recon.normalize();
    let norm_dfs = dfs_merged_recon.normalize();
    let mut merged_recon = <Reconstructor<MixedMirrorMode> as Block>::block(&[
        &[&dfs_merged_recon],
        &[&sh48_merged_recon],
    ]);
    merged_recon.truncated_pseudoinverse(vec![6]);
    println!("Merge reconstructor: {merged_recon}");

    serde_pickle::to_writer(
        &mut File::create(path.join("merged_recon_fqp.pkl"))?,
        &(&merged_recon, norm_dfs, norm_sh48),
        Default::default(),
    )?;

    // calib_sh48_rbm_bd.pseudoinverse();
    // println!("{calib_sh48_rbm_bd}");
    // calib_sh48_bm_bd.pseudoinverse();
    // println!("{calib_sh48_bm_bd}");

    /*     println!("Merging the SH48 RBMs & BMs calibration matrices");
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
    */
    // TESTING

    /* let mut dfs_processor = DispersedFringeSensorProcessing::new();
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
    }); */

    Ok(())
}

#[cfg(test)]
mod tests {
    use std::marker::PhantomData;

    use gmt_dos_clients_crseo::calibration::Modality;
    use grsim_ltao::Sh48Data;
    use interface::Data;
    use serde::Deserialize;

    use super::*;

    pub struct Model {
        sh48_om: OpticalModel<Camera<1>>,
        dfs_om: OpticalModel<DFS>,
        om: OpticalModel,
        dfs_processor: DispersedFringeSensorProcessing,
        sh48_centroids: Centroids,
        m1_rxy: Vec<Vec<f64>>,
        m1_bm: Vec<Vec<f64>>,
        dfs_rx_recon: Option<Reconstructor<CalibrationMode, ClosedLoopCalib>>,
        sh48_bm_recon: Option<Reconstructor<CalibrationMode, ClosedLoopCalib>>,
    }

    pub struct ReconstructorLoader<M, C>(PhantomData<M>, PhantomData<C>);
    impl<M: Modality + for<'a> Deserialize<'a>, C: CalibProps<M> + for<'a> Deserialize<'a>>
        ReconstructorLoader<M, C>
    {
        fn load<P: AsRef<Path>>(path: P) -> anyhow::Result<Reconstructor<M, C>> {
            Ok(serde_pickle::from_reader(
                File::open(path)?,
                Default::default(),
            )?)
        }
    }

    impl Model {
        pub fn new() -> anyhow::Result<Self> {
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
            let mut dfs_processor = DispersedFringeSensorProcessing::new();
            dfs_om_builder.initialize(&mut dfs_processor);

            let mut sh48_centroids: Centroids = Centroids::try_from(&sh48)?;
            sh48_om_builder.initialize(&mut sh48_centroids);
            dbg!(sh48_centroids.n_valid_lenslets());

            let dfs_om = dfs_om_builder.build()?;
            println!("{dfs_om}");

            let sh48_om = sh48_om_builder.build()?;
            println!("{sh48_om}");

            let om = OpticalModel::<NoSensor>::builder()
                .gmt(gmt_builder.clone())
                .build()?;

            Ok(Self {
                sh48_om,
                dfs_om,
                om,
                dfs_processor,
                sh48_centroids,
                m1_rxy: vec![vec![0.; 2]; 7],
                m1_bm: vec![vec![0.; M1_N_MODE]; 7],
                dfs_rx_recon: None,
                sh48_bm_recon: None,
            })
        }
        fn m1s_rxy_i(&mut self, sid: u8, i: usize, val: f64) -> &mut Self {
            self.m1_rxy[sid as usize - 1][i] = val;
            self
        }
        fn m1s_bm_i(&mut self, sid: u8, i: usize, val: f64) -> &mut Self {
            self.m1_bm[sid as usize - 1][i] = val;
            self
        }
        // fn rxy_to_m2<P: AsRef<Path>>(&mut self, path: P) -> anyhow::Result<&mut self> {
        fn dfs_rxy_recon(&mut self) -> anyhow::Result<&mut Self> {
            self.dfs_rx_recon = Some(ReconstructorLoader::load(
                ("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-rxy_v2.pkl"),
            ))
            .transpose()?;
            Ok(self)
        }
        // fn bm_to_m2<P: AsRef<Path>>(&mut self, path: P) -> anyhow::Result<&mut self> {
        fn sh48_bm_recon(&mut self) -> anyhow::Result<&mut Self> {
            self.sh48_bm_recon = Some(ReconstructorLoader::load(format!(
                "src/bin/agws_oiwfs/calib_sh48_{M1_N_MODE}bm.pkl"
            )))
            .transpose()?;
            Ok(self)
        }
        fn m2_rxy_cmd(&self) -> Option<Vec<f64>> {
            self.dfs_rx_recon.as_ref().map(|dfs_recon| {
                dfs_recon
                    .calib_slice()
                    .iter()
                    .zip(&self.m1_rxy)
                    .map(|(c, m1_rxy)| {
                        c.m1_to_m2() * -faer::mat::from_column_major_slice::<f64>(m1_rxy, 2, 1)
                    })
                    .flat_map(|m| m.col_as_slice(0).to_vec())
                    .chain(vec![0.; M2_N_MODE])
                    .collect()
            })
        }
        fn m2_bm_cmd(&self) -> Option<Vec<f64>> {
            self.sh48_bm_recon.as_ref().map(|sh48_recon| {
                sh48_recon
                    .calib_slice()
                    .iter()
                    .zip(&self.m1_bm)
                    .map(|(c, m1_bm)| {
                        c.m1_to_m2()
                            * -faer::mat::from_column_major_slice::<f64>(&m1_bm, M1_N_MODE, 1)
                    })
                    .flat_map(|m2_fit| {
                        let mut cmd = vec![0.];
                        cmd.extend(m2_fit.col_as_slice(0));
                        cmd
                    })
                    .collect()
            })
        }
        fn m2_cmd(&self) -> Vec<f64> {
            match (self.m2_rxy_cmd(), self.m2_bm_cmd()) {
                (Some(m2_rxy_cmd), Some(m2_bm_cmd)) => m2_rxy_cmd
                    .into_iter()
                    .zip(m2_bm_cmd.into_iter())
                    .map(|(x, y)| x + y)
                    .collect(),
                (Some(m2_rxy_cmd), None) => m2_rxy_cmd,
                (None, Some(m2_bm_cmd)) => m2_bm_cmd,
                (None, None) => vec![0.; M2_N_MODE],
            }
        }
        fn m1_rbm(&self) -> Vec<f64> {
            self.m1_rxy
                .iter()
                .cloned()
                .flat_map(|rxy| {
                    vec![0.; 3]
                        .into_iter()
                        .chain(rxy.iter().cloned())
                        .chain(Some(0.))
                        .collect::<Vec<_>>()
                })
                .collect()
        }
        fn m1_bm(&self) -> Vec<f64> {
            self.m1_bm.iter().cloned().flatten().collect()
        }
        fn update(&mut self) -> &mut Self {
            let m1_rbm = self.m1_rbm();
            let m1_bm = self.m1_bm();
            let m2_cmd = self.m2_cmd();
            let Model {
                sh48_om,
                dfs_om,
                om,
                ..
            } = self;

            <OpticalModel<NoSensor> as Read<M1RigidBodyMotions>>::read(om, m1_rbm.clone().into());
            <OpticalModel<NoSensor> as Read<M1ModeShapes>>::read(om, m1_bm.clone().into());
            <OpticalModel<NoSensor> as Read<M2ASMAsmCommand>>::read(om, m2_cmd.clone().into());
            om.update();
            println!(
                "On-axis WFE RMS {:.0?}nm",
                <OpticalModel as Write<WfeRms<-9>>>::write(om)
                    .unwrap()
                    .as_arc()
            );

            // DFS
            <OpticalModel<DFS> as Read<M1RigidBodyMotions>>::read(dfs_om, m1_rbm.clone().into());
            <OpticalModel<DFS> as Read<M1ModeShapes>>::read(dfs_om, m1_bm.clone().into());
            <OpticalModel<DFS> as Read<M2ASMAsmCommand>>::read(dfs_om, m2_cmd.clone().into());
            dfs_om.update();

            // SH48
            <OpticalModel<Camera<1>> as Read<M1RigidBodyMotions>>::read(
                sh48_om,
                m1_rbm.clone().into(),
            );
            <OpticalModel<Camera<1>> as Read<M1ModeShapes>>::read(sh48_om, m1_bm.clone().into());
            <OpticalModel<Camera<1>> as Read<M2ASMAsmCommand>>::read(
                sh48_om,
                m2_cmd.clone().into(),
            );
            sh48_om.update();

            self
        }
        fn process_dfs(&mut self) -> Data<Intercepts> {
            let Model {
                dfs_om,
                dfs_processor,
                ..
            } = self;
            <OpticalModel<DFS> as Write<DfsFftFrame<Dev>>>::write(dfs_om).map(|data| {
                <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(
                    dfs_processor,
                    data,
                )
            });
            dfs_processor.update();
            <DispersedFringeSensorProcessing as Write<Intercepts>>::write(dfs_processor).unwrap()
        }
        fn process_sh48(&mut self) -> Data<Sh48Data> {
            let Model {
                sh48_om,
                sh48_centroids,
                ..
            } = self;
            <OpticalModel<Camera<1>> as Write<Frame<Dev>>>::write(sh48_om)
                .map(|data| <Centroids as Read<Frame<Dev>>>::read(sh48_centroids, data));
            sh48_centroids.update();
            <Centroids as Write<Sh48Data>>::write(sh48_centroids).unwrap()
        }
    }

    // cargo test --release --bin merged_reconstructor -- tests::test --exact --show-output
    #[test]
    fn test() -> anyhow::Result<()> {
        let _model = Model::new()?;
        Ok(())
    }

    #[test]
    fn dfs_rxy() -> anyhow::Result<()> {
        let mut model = Model::new()?;
        let y = model
            .dfs_rxy_recon()?
            .m1s_rxy_i(1, 0, 100f64.from_mas())
            .update()
            .process_dfs();
        let mut recon: Reconstructor<MirrorMode> =
            ReconstructorLoader::load("src/bin/merged_reconstructor/calib_dfs_rbm.pkl")?;
        recon.pseudoinverse();
        println!("{recon}");

        let tt = faer::mat::from_column_major_slice::<f64>(&y.as_arc(), y.len(), 1) / &recon;

        tt[0]
            .col_as_slice(0)
            .chunks(2)
            .take(7)
            .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
            .for_each(|x| println!("{:+5.0?}", x));
        Ok(())
    }

    #[test]
    fn dfs_bm() -> anyhow::Result<()> {
        let mut model = Model::new()?;
        let y = model
            .sh48_bm_recon()?
            .m1s_bm_i(1, 0, 1e-3)
            .update()
            .process_dfs();
        let mut recon: Reconstructor<MirrorMode> =
            ReconstructorLoader::load("src/bin/merged_reconstructor/calib_dfs_bm.pkl")?;
        recon.pseudoinverse();
        println!("{recon}");

        let bm = faer::mat::from_column_major_slice::<f64>(&y.as_arc(), y.len(), 1) / &recon;

        bm[0]
            .col_as_slice(0)
            .chunks(M1_N_MODE)
            .map(|bm| bm.iter().take(6).map(|x| x * 1e4).collect::<Vec<_>>())
            .for_each(|x| println!("{:+.3?}", x));
        Ok(())
    }

    #[test]
    fn sh48_bm() -> anyhow::Result<()> {
        let mut model = Model::new()?;
        let y = model
            .sh48_bm_recon()?
            .m1s_bm_i(1, 0, 1e-4)
            .update()
            .process_sh48();
        let mut recon: Reconstructor<MirrorMode> =
            ReconstructorLoader::load("src/bin/merged_reconstructor/calib_sh48_bm.pkl")?;
        recon.pseudoinverse();
        println!("{recon}");

        let y = recon.calib_slice()[0].mask(&y.as_arc());

        let bm = faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1) / &recon;

        bm[0]
            .col_as_slice(0)
            .chunks(M1_N_MODE)
            .map(|bm| bm.iter().take(6).map(|x| x * 1e4).collect::<Vec<_>>())
            .for_each(|x| println!("{:+5.0?}", x));
        Ok(())
    }

    #[test]
    fn sh48_rxy() -> anyhow::Result<()> {
        let mut model = Model::new()?;
        let y = model
            .dfs_rxy_recon()?
            .m1s_rxy_i(1, 0, 100f64.from_mas())
            .update()
            .process_sh48();
        let mut recon: Reconstructor<MirrorMode> =
            ReconstructorLoader::load("src/bin/merged_reconstructor/calib_sh48_rbm.pkl")?;
        recon.pseudoinverse();
        println!("{recon}");

        let y = recon.calib_slice()[0].mask(&y.as_arc());

        let tt = faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1) / &recon;

        tt[0]
            .col_as_slice(0)
            .chunks(2)
            .take(7)
            .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
            .for_each(|x| println!("{:+5.0?}", x));
        Ok(())
    }

    #[test]
    fn merged() -> anyhow::Result<()> {
        let mut model = Model::new()?;
        model
            .dfs_rxy_recon()?
            .sh48_bm_recon()?
            .m1s_rxy_i(1, 0, 100f64.from_mas())
            .m1s_bm_i(1, 0, 1e-4)
            .update();
        let y_dfs = model.process_dfs().into_arc();
        let y_sh48 = model.process_sh48().into_arc();

        let (mut recon, norm_dfs, norm_sh48): (Reconstructor<MixedMirrorMode>, Vec<f64>, Vec<f64>) =
            // ReconstructorLoader::load("src/bin/merged_reconstructor/merged_recon_fqp.pkl")?;
            serde_pickle::from_reader(&mut File::open("src/bin/merged_reconstructor/merged_recon_fqp.pkl")?, Default::default())?;
        println!("{recon}");

        let m = &recon.calib_slice()[0].mask_as_slice()[y_dfs.len()..];
        let c: Calib = Calib::builder().mask(m.to_vec()).build();
        println!("{c}"); // 96768

        let y_sh48 = c.mask(&y_sh48);
        dbg!(y_sh48.len());

        let y: Vec<_> = y_dfs
            .iter()
            .map(|x| *x / norm_dfs[0])
            .chain(y_sh48.into_iter().map(|x| x / norm_sh48[0]))
            .collect();

        let v = from_column_major_slice(y.as_slice(), y.len(), 1);

        let c = v / &recon;
        let c = c[0].col_as_slice(0);
        dbg!(c.len());

        println!("RBMS");
        c[..36].chunks(6).for_each(|c| {
            println!(
                "{:+6.0?}{:+6.0?}",
                c.iter().take(3).map(|x| x * 1e9).collect::<Vec<_>>(),
                c.iter()
                    .skip(3)
                    .take(3)
                    .map(|x| x.to_mas())
                    .collect::<Vec<_>>()
            )
        });
        c[36..40].chunks(6).for_each(|c| {
            println!(
                "{:+6.0?}{:+6.0?}",
                c.iter()
                    .take(2)
                    .map(|x| x * 1e9)
                    .chain(Some(0.))
                    .collect::<Vec<_>>(),
                c.iter()
                    .skip(2)
                    .take(2)
                    .map(|x| x.to_mas())
                    .chain(Some(0.))
                    .collect::<Vec<_>>()
            )
        });
        println!("BMS");
        c[40..].chunks(M1_N_MODE).for_each(|c| {
            println!(
                "{:+7.3?}",
                c.iter().take(6).map(|x| x * 1e4).collect::<Vec<_>>()
            )
        });
        Ok(())
    }
    /*
       #[test]
       fn sh48_merged() -> anyhow::Result<()> {
           let mut model = Model::new()?;
           let y = model
               .sh48_bm_recon()?
               .m1s_bm_i(1, 0, 1e-4)
               .update()
               .process_sh48();
           let mut bm_recon: Reconstructor<MirrorMode> =
               ReconstructorLoader::load("src/bin/merged_reconstructor/calib_sh48_bm.pkl")?;
           let mut rxy_recon: Reconstructor<MirrorMode> =
               ReconstructorLoader::load("src/bin/merged_reconstructor/calib_sh48_rbm.pkl")?;
           // let mut recon0 = <Reconstructor<_> as Block>::block(&[&[&rxy_recon, &bm_recon]]);
           // rxy_recon.normalize();
           // bm_recon.normalize();

           let rxy_recon_7: Reconstructor<_> = Calib::<MirrorMode>::builder()
               .sid(7)
               .c(vec![0.; (6844 - 6080) * 12])
               .n_mode(6)
               .n_cols(12)
               // .mode(m1_rbm)
               // .mask(vec![true; 36])
               .build()
               .into();
           println!("{rxy_recon}");
           println!("{rxy_recon_7}");

           let rxy_recon = <Reconstructor<_> as Block>::block(&[&[&rxy_recon], &[&rxy_recon_7]]);
           println!("{rxy_recon}");
           let mut recon = <Reconstructor<_> as Block>::block(&[&[&rxy_recon, &bm_recon]]);
           recon.pseudoinverse();
           println!("{recon}");

           // let a_mat = recon.pinv().next().unwrap() * recon0.calib().next().unwrap();
           // let lu = a_mat.partial_piv_lu();
           // let inv = lu.inverse();

           let y = bm_recon.calib_slice()[0].mask(&y.as_arc());

           // recon
           //     .pinv()
           //     .next()
           //     .as_mut()
           //     .map(|p| p.transform(|x| &inv * x));

           let rxy_bm = faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1) / &recon;

           rxy_bm[0].col_as_slice(0)[..14]
               .chunks(2)
               .take(7)
               .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
               .for_each(|x| println!("{:+5.0?}", x));

           rxy_bm[0].col_as_slice(0)[14..]
               .chunks(M1_N_MODE)
               .map(|bm| bm.iter().take(6).map(|x| x * 1e4).collect::<Vec<_>>())
               .for_each(|x| println!("{:+5.0?}", x));
           Ok(())
       }
    */
    /*     #[test]
       fn dfs_rxy() -> anyhow::Result<()> {
           let Model {
               sh48_om,
               dfs_om,
               om,
               dfs_processor,
               sh48_centroids,
               ..
           } = &mut Model::new()?;

           let dfs_recon: Reconstructor<CalibrationMode, ClosedLoopCalib> = serde_pickle::from_reader(
               File::open("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-rxy_v2.pkl")?,
               Default::default(),
           )?;

           let mut m1_rxy = vec![vec![0f64; 2]; 7];
           m1_rxy[0][0] = 0.1f64.from_arcsec();
           m1_rxy[1][1] = 0.1f64.from_arcsec();
           let m1_rxy_cmd: Vec<_> = dfs_recon
               .calib_slice()
               .iter()
               .zip(&m1_rxy)
               .map(|(c, m1_rxy)| {
                   c.m1_to_m2() * -faer::mat::from_column_major_slice::<f64>(m1_rxy, 2, 1)
               })
               .flat_map(|m| m.col_as_slice(0).to_vec())
               .chain(vec![0.; M2_N_MODE])
               .collect();

           let mut m1_bm = vec![vec![0f64; M1_N_MODE]; 7];
           // m1_bm[0][0] = 1e-4;
           // m1_bm[6][4] = 1e-5;
           // let m1_bm_cmd: Vec<_> = dfs_recon
           //     .calib_slice()
           //     .iter()
           //     .zip(&m1_bm)
           //     .map(|(c, m1_bm)| {
           //         c.m1_to_m2() * -faer::mat::from_column_major_slice::<f64>(&m1_bm, M1_N_MODE, 1)
           //     })
           //     .flat_map(|m2_fit| {
           //         let mut cmd = vec![0.];
           //         cmd.extend(m2_fit.col_as_slice(0));
           //         cmd
           //     })
           //     .collect();
           let cmd: Vec<_> = m1_rxy_cmd;
           // .into_iter()
           // .zip(m1_bm_cmd.into_iter())
           // .map(|(x, y)| x + y)
           // .collect();
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

           // dbg!(&cmd[..10]);
           <OpticalModel<NoSensor> as Read<M1RigidBodyMotions>>::read(om, m1_rbm.clone().into());
           <OpticalModel<NoSensor> as Read<M1ModeShapes>>::read(
               om,
               m1_bm.iter().cloned().flatten().collect::<Vec<_>>().into(),
           );
           <OpticalModel<NoSensor> as Read<M2ASMAsmCommand>>::read(om, cmd.clone().into());
           om.update();
           println!(
               "On-axis WFE RMS {:0?}nm",
               <OpticalModel as Write<WfeRms<-9>>>::write(om)
                   .unwrap()
                   .as_arc()
           );

           // DFS
           <OpticalModel<DFS> as Read<M1RigidBodyMotions>>::read(dfs_om, m1_rbm.clone().into());
           <OpticalModel<DFS> as Read<M1ModeShapes>>::read(
               dfs_om,
               m1_bm.iter().cloned().flatten().collect::<Vec<_>>().into(),
           );
           <OpticalModel<DFS> as Read<M2ASMAsmCommand>>::read(dfs_om, cmd.clone().into());

           dfs_om.update();

           <OpticalModel<DFS> as Write<DfsFftFrame<Dev>>>::write(dfs_om).map(|data| {
               <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(dfs_processor, data)
           });
           dfs_processor.update();
           let dfs_y =
               <DispersedFringeSensorProcessing as Write<Intercepts>>::write(dfs_processor).unwrap();
           dbg!(dfs_y.len());
           let mut dfs_recon = dfs_recon.collapse();
           dfs_recon.pseudoinverse();
           // println!("{dfs_recon}");

           // // let q = calib_dfs_rbm_col.normalize();
           // calib_dfs_rbm_col.pseudoinverse();
           // println!("{calib_dfs_rbm_col}");

           // dfs_merged_recon0.pseudoinverse();
           // println!("{dfs_merged_recon0}");

           let tt = faer::mat::from_column_major_slice::<f64>(&dfs_y.as_arc(), 36, 1) / &dfs_recon;

           tt[0]
               .col_as_slice(0)
               .chunks(2)
               .take(7)
               .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
               .for_each(|x| println!("{:+5.0?}", x));

           Ok(())
       }
    */
}
