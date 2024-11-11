//! OIWFS & Global Tip-Tilt

use crseo::{gmt::GmtM1, imaging::Detector, FromBuilder, Gmt, Source};
use gmt_dos_clients::gif::Frame as Png;
use gmt_dos_clients_crseo::{
    calibration::{algebra::CalibProps, estimation::Estimation, GlobalCalibration, Reconstructor},
    centroiding::CentroidsProcessing,
    sensors::Camera,
    DeviceInitialize, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    optics::{Dev, Frame, Host, M1GlobalTipTilt, M2GlobalTipTilt, SensorData},
    Estimate,
};
use interface::{Read, Update, Write};
use skyangle::Conversion;

const M1_N_MODE: usize = 27;
const M2_N_MODE: usize = 66;

fn main() -> anyhow::Result<()> {
    let gmt_builder = Gmt::builder()
        .m1("bending modes", M1_N_MODE)
        .m2("Karhunen-Loeve", M2_N_MODE)
        .m1_truss_projection(false);

    let oiwfs = Camera::builder().detector(Detector::default().n_px_imagelet(128).osf(4));
    let mut oiwfs_centroids: CentroidsProcessing = CentroidsProcessing::try_from(&oiwfs)?;

    let oiwfs_tt_om_builder = OpticalModel::<Camera<1>>::builder()
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("K"))
        .sensor(oiwfs);

    let mut recon = <CentroidsProcessing as GlobalCalibration<GmtM1>>::calibrate(
        &(&oiwfs_tt_om_builder).into(),
        gmt_dos_clients_crseo::calibration::CalibrationMode::GlobalTipTilt(100f64.from_mas()),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    oiwfs_tt_om_builder.initialize(&mut oiwfs_centroids);
    dbg!(oiwfs_centroids.n_valid_lenslets());

    let mut oiwfs_tt_om = oiwfs_tt_om_builder
        // .atmosphere(atm_builder.clone())
        .build()?;
    println!("{oiwfs_tt_om}");

    <OpticalModel<_> as Read<M1GlobalTipTilt>>::read(
        &mut oiwfs_tt_om,
        vec![100f64.from_mas(), 100f64.from_mas()].into(),
    );
    oiwfs_tt_om.update();

    let mut frame = Png::<f32>::new("oiwfs.png", 128);
    <OpticalModel<_> as Write<Frame<Host>>>::write(&mut oiwfs_tt_om)
        .map(|data| <Png<_> as Read<Frame<Host>>>::read(&mut frame, data));
    frame.update();
    frame.save()?;

    <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut oiwfs_tt_om)
        .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut oiwfs_centroids, data));
    oiwfs_centroids.update();
    <CentroidsProcessing as Write<SensorData>>::write(&mut oiwfs_centroids).map(|data| {
        println!("s_xy: {:?}mas", data.iter().map(|x| x).collect::<Vec<_>>());
        <Reconstructor as Read<SensorData>>::read(&mut recon, data);
    });
    recon.update();
    let estimate = <Reconstructor as Write<Estimate>>::write(&mut recon)
        .unwrap()
        .into_arc();
    println!(
        "Estimate: {:?}",
        estimate.iter().map(|x| x.to_mas()).collect::<Vec<_>>()
    );

    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut oiwfs_tt_om, vec![0f64; 42].into());
    oiwfs_tt_om.update();
    <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut oiwfs_tt_om)
        .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut oiwfs_centroids, data));

    oiwfs_centroids.update();
    <CentroidsProcessing as Write<SensorData>>::write(&mut oiwfs_centroids)
        .map(|data| println!("s_xy: {:?}mas", data.iter().map(|x| x).collect::<Vec<_>>()));

    Ok(())
}
