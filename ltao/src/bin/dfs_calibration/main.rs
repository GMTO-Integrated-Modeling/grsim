use std::fs::File;

use crseo::{FromBuilder, Gmt, Source};
use gmt_dos_clients_crseo::{
    calibration::{Calib, CalibrationMode, Reconstructor},
    sensors::{DispersedFringeSensor, DispersedFringeSensorProcessing, NoSensor},
    DeviceInitialize, OpticalModel,
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    /*     let mut calib_m2_modes_onaxis: Reconstructor = serde_pickle::from_reader(
        &mut std::fs::File::open("src/bin/calibration/calib_m2_modes_onaxis.pkl")?,
        Default::default(),
    )?;
    let mut calib_m1_rbm_onaxis: Reconstructor = serde_pickle::from_reader(
        &mut std::fs::File::open("src/bin/calibration/calib_m1_rbm_onaxis.pkl")?,
        Default::default(),
    )?;
    calib_m1_rbm_onaxis.match_areas(&mut calib_m2_modes_onaxis);
    println!("{calib_m2_modes_onaxis}");
    println!("{calib_m1_rbm_onaxis}");

    let m1_to_m2 = calib_m2_modes_onaxis.least_square_solve(&calib_m1_rbm_onaxis);
    serde_pickle::to_writer(
        &mut std::fs::File::create("src/bin/dfs_calibration/m1_to_m2.pkl")?,
        &m1_to_m2,
        Default::default(),
    )?; */
    let mut m1_to_m2: Vec<faer::Mat<f64>> = serde_pickle::from_reader(
        &mut std::fs::File::open("src/bin/dfs_calibration/m1_to_m2.pkl")?,
        Default::default(),
    )?;
    m1_to_m2
        .iter()
        .enumerate()
        .for_each(|(i, m1_to_m2)| println!("M1->M2 ({},{})", m1_to_m2.nrows(), m1_to_m2.ncols()));

    let agws_gs_builder = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let gmt_builder = Gmt::builder().m2("Karhunen-Loeve", m1_to_m2[0].nrows());

    let mut dfs_om_builder = OpticalModel::<DFS>::builder()
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone())
        .sensor(
            DFS::builder()
                .source(agws_gs_builder.clone())
                .nyquist_factor(3.),
        );

    let mut dfs_processor = DispersedFringeSensorProcessing::new();
    dfs_om_builder.initialize(&mut dfs_processor);

    let mut dfs_om = dfs_om_builder.build()?;
    println!("{dfs_om}");

    let mut om = OpticalModel::<NoSensor>::builder()
        .gmt(gmt_builder.clone())
        .build()?;
    println!("{om}");

    let mut y = vec![];

    for (i, m1_to_m2) in m1_to_m2.iter().enumerate().take(6) {
        for (c, rxy) in m1_to_m2.col_iter().zip([3, 4].into_iter()) {
            let mut m1_rbm = vec![vec![0f64; 6]; 7];
            m1_rbm[i][rxy] = 1f64.from_arcsec();
            <OpticalModel as Read<M1RigidBodyMotions>>::read(
                &mut om,
                m1_rbm.iter().flatten().cloned().collect::<Vec<_>>().into(),
            );
            <OpticalModel<DFS> as Read<M1RigidBodyMotions>>::read(
                &mut dfs_om,
                m1_rbm.iter().flatten().cloned().collect::<Vec<_>>().into(),
            );

            let mut cmd = vec![vec![0f64; 66]; 7];
            cmd[i] = c
                .iter()
                .map(|x| x * -1f64.from_arcsec())
                .collect::<Vec<_>>();
            <OpticalModel as Read<M2ASMAsmCommand>>::read(
                &mut om,
                cmd.iter().flatten().cloned().collect::<Vec<_>>().into(),
            );
            <OpticalModel<DFS> as Read<M2ASMAsmCommand>>::read(
                &mut dfs_om,
                cmd.iter().flatten().cloned().collect::<Vec<_>>().into(),
            );

            om.update();
            dfs_om.update();
            <OpticalModel as Write<WfeRms<-9>>>::write(&mut om)
                .zip(<OpticalModel<DFS> as Write<WfeRms<-9>>>::write(&mut dfs_om))
                .map(|(on, off)| {
                    println!(
                        "SID {}: {:.0?} {:.0?}",
                        i + 1,
                        on.into_arc(),
                        off.into_arc(),
                    )
                });

            <OpticalModel<DFS> as Write<DfsFftFrame<Dev>>>::write(&mut dfs_om).map(|data| {
                <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(
                    &mut dfs_processor,
                    data,
                )
            });
            dfs_processor.update();
            let data =
                <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor)
                    .unwrap();

            y.extend(data.into_arc().iter().map(|x| x.to_arcsec()));
        }
    }

    dbg!(y.len());
    // y.chunks(36).for_each(|y| println!("{:+.6?}", y));

    let calib = Calib::builder()
        .c(y)
        .n_mode(6)
        .n_cols(12)
        .mode(CalibrationMode::RBM([
            None,                     // Tx
            None,                     // Ty
            None,                     // Tz
            Some(1f64.from_arcsec()), // Rx
            Some(1f64.from_arcsec()), // Ry
            None,                     // Rz
        ]))
        .mask(vec![true; 36])
        .build();
    let mut recon = Reconstructor::from(calib);
    recon.pseudoinverse();
    print!("{recon}");
    serde_pickle::to_writer(
        &mut File::create("src/bin/calibration/calib_dfs_closed-loop_m1-rxy.pkl")?,
        &recon,
        Default::default(),
    )?;

    let mut m1_rxy = vec![vec![0f64; 2]; 7];
    m1_rxy[0][0] = 1f64.from_arcsec();
    m1_rxy[1][1] = 1f64.from_arcsec();
    let cmd: Vec<_> = m1_to_m2
        .iter()
        .zip(&m1_rxy)
        .map(|(m1_to_m2, m1_rxy)| {
            m1_to_m2 * -faer::mat::from_column_major_slice::<f64>(m1_rxy, 2, 1)
        })
        .flat_map(|m| m.col_as_slice(0).to_vec())
        .collect();
    println!("{:?}", cmd.len());
    // let q = &recon * vec![c];

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

    <OpticalModel<DFS> as Read<M1RigidBodyMotions>>::read(&mut dfs_om, m1_rbm.into());
    // dbg!(&cmd[..10]);
    // dbg!(&cmd[66..76]);
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
    let tt = faer::mat::from_column_major_slice::<f64>(&y, 36, 1) / &recon;

    tt[0]
        .col_as_slice(0)
        .chunks(2)
        .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .for_each(|x| println!("{:+5.0?}", x));

    Ok(())
}
