use std::error::Error;

use crseo::{FromBuilder, Gmt, Source};
use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
use skyangle::Conversion;

use gmt_dos_clients_crseo::{calibration::Calibrate, sensors::WaveSensor};

use crseo::gmt::{GmtM1, GmtM2};

use gmt_dos_clients_crseo::{
    calibration::{
        estimation::{closed_loop::ClosedLoopEstimation, Estimation},
        MirrorMode, Reconstructor,
    },
    sensors::DispersedFringeSensor,
    DispersedFringeSensorProcessing, OpticalModel,
};

use gmt_dos_clients_crseo::calibration::{CalibrationMode, ClosedLoopCalibrate};

type DFS = DispersedFringeSensor<1, 1>;

#[test]
fn closed_loop_calibrate() -> Result<(), Box<dyn Error>> {
    let m2_n_mode = 66;
    let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
    let optical_model = OpticalModel::<DFS>::builder()
        .gmt(gmt.clone())
        .source(agws_gs.clone())
        .sensor(DFS::builder().source(agws_gs.clone().band("J")));
    let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());

    let mut recon =
        <DispersedFringeSensorProcessing as ClosedLoopCalibrate<WaveSensor>>::calibrate_serial(
            &optical_model,
            MirrorMode::from(CalibrationMode::RBM([
                None,                    // Tx
                None,                    // Ty
                None,                    // Tz
                Some(100f64.from_mas()), // Rx
                Some(100f64.from_mas()), // Ry
                None,                    // Rz
            ]))
            .update((7, CalibrationMode::empty_rbm())),
            &closed_loop_optical_model,
            CalibrationMode::modes(m2_n_mode, 1e-6),
        )?;
    recon.pseudoinverse();
    println!("{recon}");

    let mut data = vec![0.; 42];
    data[3] = 100f64.from_mas();
    data[6 * 1 + 4] = 100f64.from_mas();
    let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
        WaveSensor,
        M1RigidBodyMotions,
    >>::estimate(&optical_model, &closed_loop_optical_model, &mut recon, data)?;
    estimate
        .chunks(6)
        .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    Ok(())
}

#[test]
fn calibrate_tz() -> Result<(), Box<dyn Error>> {
    let m2_n_mode = 66;
    let agws_gs = Source::builder()
        .size(3)
        .on_ring(6f32.from_arcmin())
        .band("J");
    let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
    let optical_model = OpticalModel::<DFS>::builder()
        .gmt(gmt.clone())
        .source(agws_gs.clone())
        .sensor(DFS::builder().source(agws_gs));

    let mut recon = <DispersedFringeSensorProcessing as Calibrate<GmtM1>>::calibrate(
        &optical_model,
        CalibrationMode::t_z(1e-6),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    println!("Tz estimation from Tz:");
    let mut data = vec![0.; 42];
    data[2] = 1e-6;
    let estimate = <DispersedFringeSensorProcessing as Estimation<M1RigidBodyMotions>>::estimate(
        &optical_model,
        &mut recon,
        data,
    )?;
    estimate
        .chunks(6)
        .map(|c| c.iter().map(|x| x * 1e6).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    Ok(())
}
#[test]
fn closed_loop_calibrate_7() -> Result<(), Box<dyn Error>> {
    let m2_n_mode = 66;
    let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
    let optical_model = OpticalModel::<DFS>::builder()
        .gmt(gmt.clone())
        .source(agws_gs.clone())
        .sensor(DFS::builder().source(agws_gs.clone().band("J")));
    let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());

    let closed_loop_calib_mode = CalibrationMode::modes(m2_n_mode, 1e-6);
    let mut m2_to_closed_loop_sensor: Reconstructor = <WaveSensor as Calibrate<GmtM2>>::calibrate(
        &closed_loop_optical_model,
        closed_loop_calib_mode.clone(),
    )?;
    m2_to_closed_loop_sensor.pseudoinverse();

    let mut recon =
        <DispersedFringeSensorProcessing as ClosedLoopCalibrate<WaveSensor>>::calibrate_serial(
            &optical_model,
            MirrorMode::from(CalibrationMode::RBM([
                None,                    // Tx
                None,                    // Ty
                None,                    // Tz
                Some(100f64.from_mas()), // Rx
                Some(100f64.from_mas()), // Ry
                None,                    // Rz
            ]))
            .update((7, CalibrationMode::empty_rbm())),
            &closed_loop_optical_model,
            closed_loop_calib_mode,
        )?;
    recon.pseudoinverse();
    println!("{recon}");

    let mut data = vec![0.; 42];
    data[36 + 3] = 100f64.from_mas();
    let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
        WaveSensor,
        M1RigidBodyMotions,
    >>::estimate_with_closed_loop_reconstructor(
        &optical_model,
        &closed_loop_optical_model,
        &mut recon,
        data,
        m2_to_closed_loop_sensor,
    )?;
    estimate
        .chunks(6)
        .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    Ok(())
}
