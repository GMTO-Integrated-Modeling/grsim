use std::fs::File;

use crseo::{FromBuilder, Gmt, Source};
use gmt_dos_clients_crseo::{
    calibration::{
        algebra::Collapse, CalibrationMode, ClosedLoopCalibration, ClosedLoopReconstructor,
        Reconstructor,
    },
    sensors::{DispersedFringeSensor, NoSensor, WaveSensor},
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
use interface::{filing::Filing, Read, Update, Write};
use skyangle::Conversion;

type DFS = DispersedFringeSensor<1, 1>;
type DFS11 = DispersedFringeSensor;

const M2_N_MODE: usize = 66;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let agws_gs_builder = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let gmt_builder = Gmt::builder()
        // .m1("bending modes", 21)
        .m2("Karhunen-Loeve", M2_N_MODE);

    let dfs_om_builder = OpticalModel::<DFS>::builder()
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone())
        .sensor(
            DFS::builder()
                .source(agws_gs_builder.clone())
                .nyquist_factor(3.),
        );

    let recon7 = Reconstructor::from_path_or_else(
        "src/bin/dfs_calibration/calib_dfs_closed-loop_m1-rxy7_v2.pkl",
        || {
            <DispersedFringeSensorProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate_serial(
                &dfs_om_builder,
                [CalibrationMode::RBM([
                    None,                     // Tx
                    None,                     // Ty
                    None,                     // Tz
                    Some(1f64.from_arcsec()), // Rx
                    Some(1f64.from_arcsec()), // Ry
                    None,                     // Rz
                ]); 7],
                // CalibrationMode::modes(21, 1e-6),
                &OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
                CalibrationMode::modes(M2_N_MODE, 1e-6),
            )
            .unwrap()
        },
    )?;
    println!("{recon7}");

    /*     let mut recon =
        <DispersedFringeSensorProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate_serial(
            &dfs_om_builder,
            [CalibrationMode::RBM([
                None,                     // Tx
                None,                     // Ty
                None,                     // Tz
                Some(1f64.from_arcsec()), // Rx
                Some(1f64.from_arcsec()), // Ry
                None,                     // Rz
            ]); 6],
            // CalibrationMode::modes(21, 1e-6),
            &OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
            CalibrationMode::modes(M2_N_MODE, 1e-6),
        )?;
    // recon.pseudoinverse();
    println!("{recon}");
    serde_pickle::to_writer(
        &mut File::create("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-rxy_v2.pkl")?,
        // &mut File::create("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-21bm.pkl")?,
        &recon,
        Default::default(),
    )?; */

    let recon: ClosedLoopReconstructor = serde_pickle::from_reader(
        File::open("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-rxy_v2.pkl")?,
        Default::default(),
    )?;

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
    let tt = faer::mat::from_column_major_slice::<f64>(&y, 36, 1) / &recon;

    tt[0]
        .col_as_slice(0)
        .chunks(2)
        .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .for_each(|x| println!("{:+5.0?} ({:5.0})", x, x[0].hypot(x[1])));

    Ok(())
}

#[cfg(test)]
mod tests {
    #[test]
    fn diff_mat() {
        // let r0 = [1, -1, 0, 0, 0, 0, 0];
        let rs: Vec<_> = (0..6)
            .flat_map(|i| {
                let mut r0 = vec![1., -1., 0., 0., 0., 0.];
                r0.rotate_right(i);
                r0.push(0.);
                println!("{:?}", r0);
                r0
            })
            .chain((0..6).flat_map(|i| {
                let mut r0 = vec![0., 0., 0., 0., 0., 0., -1.];
                r0[i] = 1.;
                println!("{:?}", r0);
                r0
            }))
            .collect();

        let diff_mat = faer::mat::from_row_major_slice::<f64>(&rs, 12, 7);
        println!("{:?}", diff_mat);
        let svd = diff_mat.svd();
        let s = svd.s_diagonal();
        dbg!(&s);
    }
}
