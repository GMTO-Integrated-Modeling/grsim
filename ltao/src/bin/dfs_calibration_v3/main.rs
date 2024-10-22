use std::fs::File;

use crseo::{FromBuilder, Gmt, Source};
use gmt_dos_clients_crseo::{
    calibration::{
        algebra::{CalibProps, Collapse}, CalibrationMode, ClosedLoopCalib, ClosedLoopCalibration,
        Reconstructor,
    },
    sensors::{DispersedFringeSensor, NoSensor, SegmentPistonSensor, WaveSensor},
    DeviceInitialize, DispersedFringeSensorProcessing, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, WfeRms,
    },
};
use interface::{Read, Update, Write};
use skyangle::Conversion;

type DFS = DispersedFringeSensor<1, 1>;
type DFS11 = DispersedFringeSensor;

const M2_N_MODE: usize = 66;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let agws_gs_builder = Source::builder()
        .band("J")
        .size(3)
        .on_ring(6f32.from_arcmin());
    let gmt_builder = Gmt::builder()
        // .m1("bending modes", 21)
        .m2("Karhunen-Loeve", M2_N_MODE);

    let dfs_om_builder = OpticalModel::<DFS>::builder()
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone())
        .sensor(DFS::builder().source(agws_gs_builder.clone().band("J")));

    let sp_om_builder = OpticalModel::<SegmentPistonSensor>::builder()
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone());

    let recon_closedloop_rxy_to_sp: Reconstructor<CalibrationMode, ClosedLoopCalib> =
        if let Ok(mut file) =
            File::open("src/bin/dfs_calibration_v3/recon_closedloop_rxy_to_sp.pkl")
        {
            serde_pickle::from_reader(&mut file, Default::default())?
        } else {
            println!("recon_closedloop_rxy_to_sp");
            let recon = <SegmentPistonSensor as ClosedLoopCalibration<WaveSensor>>::calibrate(
                &sp_om_builder,
                [CalibrationMode::RBM([
                    None,                    // Tx
                    None,                    // Ty
                    None,                    // Tz
                    Some(100f64.from_mas()), // Rx
                    Some(100f64.from_mas()), // Ry
                    None,                    // Rz
                ]); 7],
                // CalibrationMode::modes(21, 1e-6),
                &OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
                CalibrationMode::modes(M2_N_MODE, 1e-6),
            )?;
            serde_pickle::to_writer(
                &mut File::create("src/bin/dfs_calibration_v3/recon_closedloop_rxy_to_sp.pkl")?,
                &recon,
                Default::default(),
            )?;
            recon
        };
    println!("{recon_closedloop_rxy_to_sp}");

    let recon7 = if let Ok(mut file) =
        File::open("src/bin/dfs_calibration_v3/calib_dfs_closed-loop_m1-rxy7.pkl")
    {
        serde_pickle::from_reader(&mut file, Default::default())?
    } else {
        println!("calib_dfs_closed-loop_m1-rxy7");
        let recon =
            <DispersedFringeSensorProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate_serial(
                &dfs_om_builder,
                [CalibrationMode::RBM([
                    None,                    // Tx
                    None,                    // Ty
                    None,                    // Tz
                    Some(100f64.from_mas()), // Rx
                    Some(100f64.from_mas()), // Ry
                    None,                    // Rz
                ]); 7],
                // CalibrationMode::modes(21, 1e-6),
                &OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
                CalibrationMode::modes(M2_N_MODE, 1e-6),
            )?;
        serde_pickle::to_writer(
            &mut File::create("src/bin/dfs_calibration_v3/calib_dfs_closed-loop_m1-rxy7.pkl")?,
            &recon,
            Default::default(),
        )?;
        recon
    };
    println!("{recon7}");

    let recon = if let Ok(mut file) =
        File::open("src/bin/dfs_calibration_v3/recon_dfs_closed-loop_m1-rxy.pkl")
    {
        serde_pickle::from_reader(&mut file, Default::default())?
    } else {
        println!("recon_dfs_closed-loop_m1-rxy");
        let mut recon =
            <DispersedFringeSensorProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate_serial(
                &dfs_om_builder,
                [CalibrationMode::RBM([
                    None,                    // Tx
                    None,                    // Ty
                    None,                    // Tz
                    Some(100f64.from_mas()), // Rx
                    Some(100f64.from_mas()), // Ry
                    None,                    // Rz
                ]); 6],
                // CalibrationMode::modes(21, 1e-6),
                &OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
                CalibrationMode::modes(M2_N_MODE, 1e-6),
            )?;
        recon.pseudoinverse();
        serde_pickle::to_writer(
            &mut File::create("src/bin/dfs_calibration_v3/recon_dfs_closed-loop_m1-rxy.pkl")?,
            &recon,
            Default::default(),
        )?;
        recon
    };
    println!("{recon}");

    let mut dfs_processor = DispersedFringeSensorProcessing::new();
    dfs_om_builder.initialize(&mut dfs_processor);

    let mut dfs_om = dfs_om_builder.build()?;
    println!("{dfs_om}");

    let mut m1_rxy = vec![vec![0f64; 2]; 7];
    m1_rxy[0][0] = 100f64.from_mas();
    // m1_rxy[1][1] = 1f64.from_arcsec();
    let cmd: Vec<_> = recon7
        .calib_slice()
        .iter()
        .zip(&m1_rxy)
        .map(|(c, m1_rxy)| c.m1_to_m2() * -faer::mat::from_column_major_slice::<f64>(m1_rxy, 2, 1))
        .flat_map(|m| m.col_as_slice(0).to_vec())
        // .chain(vec![0.; M2_N_MODE])
        .collect();
    println!("cmds: {:?}", cmd.len());
    // let q = &recon * vec![c];

    let m1_rbm: Vec<f64> = m1_rxy
        .into_iter()
        .flat_map(|rxy| {
            vec![0.; 3]
                .into_iter()
                .chain(rxy.into_iter())
                .chain(Some(0.))
                .collect::<Vec<_>>()
        })
        .collect();
    println!("rbm: {:?}", m1_rbm.len());

    let mut om = OpticalModel::<NoSensor>::builder()
        .gmt(gmt_builder.clone())
        .build()?;
    // dbg!(&cmd[..10]);
    <OpticalModel<NoSensor> as Read<M1RigidBodyMotions>>::read(&mut om, m1_rbm.clone().into());
    <OpticalModel<NoSensor> as Read<M2ASMAsmCommand>>::read(&mut om, cmd.clone().into());
    om.update();
    dbg!(<OpticalModel as Write<WfeRms<-9>>>::write(&mut om));

    <OpticalModel<DFS> as Read<M1RigidBodyMotions>>::read(&mut dfs_om, m1_rbm.into());
    dbg!(&cmd[..10]);
    dbg!(&cmd[66..76]);
    <OpticalModel<DFS> as Read<M2ASMAsmCommand>>::read(&mut dfs_om, cmd.into());

    dfs_om.update();

    <OpticalModel<DFS> as Write<DfsFftFrame<Dev>>>::write(&mut dfs_om).map(|data| {
        <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(&mut dfs_processor, data)
    });
    dfs_processor.update();
    let y = <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor)
        .unwrap()
        .into_arc();
    dbg!(y.len());
    let mut recon = recon.collapse();
    recon.pseudoinverse();
    println!("{recon}");
    recon.eyes_check(None);
    let tt = faer::mat::from_column_major_slice::<f64>(&y, 36, 1) / &recon;

    tt[0]
        .col_as_slice(0)
        .chunks(2)
        .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .for_each(|x| println!("{:+5.0?} ({:5.0})", x, x[0].hypot(x[1])));

    /*     // CONSTRAINED RECON

    let recon7_col = recon7.collapse();
    let mat_d = recon7_col.calib_slice()[0].mat_ref();
    dbg!(mat_d.shape());

    let recon_closedloop_rxy_to_sp_bd = recon_closedloop_rxy_to_sp.diagonal();
    let mat_h = from_row_major_slice(&vec![1.; 7], 1, 7)
        * recon_closedloop_rxy_to_sp_bd.calib_slice()[0].mat_ref();
    dbg!(mat_h.shape());

    let mut mat_c = Mat::zeros(51, 51);
    let mut dst = mat_c.as_mut().submatrix_mut(14, 0, 36, 14);
    dst.copy_from(mat_d);
    let mut dst = mat_c.as_mut().submatrix_mut(50, 0, 1, 14);
    dst.copy_from(mat_h);
    mat_c += mat_c.clone().transpose();
    let mut dst = mat_c.as_mut().submatrix_mut(14, 14, 36, 36);
    dst.copy_from(Mat::<f64>::identity(36, 36));

    println!("{:?}", mat_c.shape());

    let svd = mat_c.svd();
    // dbg!(svd.s_diagonal().column_vector_as_diagonal().shape());

    let u = svd.u().subrows(14, 36);
    dbg!(u.shape());
    let v = svd.v().subrows(0, 14);
    dbg!(v.shape());

    let mat_m = v * svd.s_diagonal().column_vector_as_diagonal() * u.transpose();
    println!("{:?}", mat_m.shape());

    let tt = mat_m * from_column_major_slice::<f64>(&y, 36, 1);
    tt.col_as_slice(0)
        .chunks(2)
        .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .for_each(|x| println!("{:+5.0?} ({:5.0})", x, x[0].hypot(x[1]))); */

    Ok(())
}
