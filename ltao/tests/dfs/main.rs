use std::error::Error;

use crseo::{FromBuilder, Gmt, Source};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{dispersed_fringe_sensor::Intercepts, Wavefront},
};
use skyangle::Conversion;

use gmt_dos_clients_crseo::{
    calibration::{
        algebra::{Block, Collapse, Merge},
        correction::{ClosedLoopCorrection, Correction},
        Calibration,
    },
    sensors::{NoSensor, WaveSensor},
};

use crseo::gmt::{GmtM1, GmtM2};

use gmt_dos_clients_crseo::{
    calibration::{
        estimation::{closed_loop::ClosedLoopEstimation, Estimation},
        MirrorMode, Reconstructor,
    },
    sensors::DispersedFringeSensor,
    DispersedFringeSensorProcessing, OpticalModel,
};

use gmt_dos_clients_crseo::calibration::{CalibrationMode, ClosedLoopCalibration};

use interface::{Read, Update, Write};

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
        <DispersedFringeSensorProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate_serial(
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
    >>::estimate(
        &optical_model,
        &closed_loop_optical_model,
        &mut recon,
        &data,
    )?;
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

    let mut recon = <DispersedFringeSensorProcessing as Calibration<GmtM1>>::calibrate(
        &optical_model,
        CalibrationMode::t_z(1e-6),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    let mut data = vec![0.; 42];
    data[36 + 2] = 1e-6;
    let estimate = <DispersedFringeSensorProcessing as Estimation<M1RigidBodyMotions>>::estimate(
        &optical_model,
        &mut recon,
        &data,
    )?;
    estimate
        .chunks(6)
        .map(|c| c.iter().map(|x| x * 1e6).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    Ok(())
}

#[test]
fn calibrate_rxy_tz() -> Result<(), Box<dyn Error>> {
    let m2_n_mode = 66;
    let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
    let optical_model = OpticalModel::<DFS>::builder()
        .gmt(gmt.clone())
        .source(agws_gs.clone())
        .sensor(DFS::builder().source(agws_gs.clone().band("J")));
    let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());

    println!("Rxy estimation");
    let mut recon_rxy =
        <DispersedFringeSensorProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate_serial(
            &optical_model,
            MirrorMode::from(CalibrationMode::RBM([
                None,                    // Tx
                None,                    // Ty
                None,                    // Tz
                Some(100f64.from_mas()), // Rx
                Some(100f64.from_mas()), // Ry
                None,                    // Rz
            ]))
            // .remove(7),
            .update((7, CalibrationMode::empty_rbm())),
            &closed_loop_optical_model,
            CalibrationMode::modes(m2_n_mode, 1e-6),
        )?;
    recon_rxy.pseudoinverse();
    println!("{recon_rxy}");

    let mut data = vec![0.; 42];
    data[3] = 100f64.from_mas();
    data[6 * 1 + 4] = 100f64.from_mas();
    data[2] = 1e-6;
    let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
        WaveSensor,
        M1RigidBodyMotions,
    >>::estimate(
        &optical_model,
        &closed_loop_optical_model,
        &mut recon_rxy,
        &data,
    )?;
    estimate
        .chunks(6)
        .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    let m2_closed_loop_reconstructor = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
        WaveSensor,
        M1RigidBodyMotions,
    >>::closed_loop_reconstructor(&mut recon_rxy);

    println!("Tz estimation");
    let mut recon_tz = <DispersedFringeSensorProcessing as Calibration<GmtM1>>::calibrate(
        &optical_model,
        MirrorMode::from(CalibrationMode::RBM([
            None,
            None,
            Some(1e-6), // Txyz
            None,
            None,
            None, // Rxyz
        ]))
        .remove(7), // .update((7, CalibrationMode::empty_rbm())),
    )?;
    recon_tz.pseudoinverse();
    println!("{recon_tz}");

    let mut data = vec![0.; 42];
    data[3] = 100f64.from_mas();
    data[6 * 1 + 4] = 100f64.from_mas();
    data[2] = 1e-6;
    let estimate = <DispersedFringeSensorProcessing as Estimation<M1RigidBodyMotions>>::estimate(
        &optical_model,
        &mut recon_tz,
        &data,
    )?;
    estimate
        .chunks(6)
        .map(|c| c.iter().map(|x| x * 1e6).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    let mut dfs_processor = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
        WaveSensor,
        M1RigidBodyMotions,
    >>::processor(
        &optical_model,
        &closed_loop_optical_model,
        &data,
        m2_closed_loop_reconstructor,
    )?;

    // let recon_tz = recon.collapse();
    let mut recon = recon_rxy;
    recon.merge(recon_tz);
    let mut recon = recon.collapse();
    recon.pseudoinverse();
    println!("{recon}");

    let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
        WaveSensor,
        M1RigidBodyMotions,
    >>::recon(&mut dfs_processor, &mut recon)?;

    estimate
        .chunks(6)
        .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    Ok(())
}

#[test]
fn closed_loop_calibrate_7() -> Result<(), Box<dyn Error>> {
    let m2_n_mode = 66;
    let agws_gs = Source::builder()
        .size(3)
        .on_ring(6f32.from_arcmin())
        .band("J");
    let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
    let optical_model = OpticalModel::<DFS>::builder()
        .gmt(gmt.clone())
        .source(agws_gs.clone())
        .sensor(DFS::builder().source(agws_gs.clone()));
    let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());

    let closed_loop_calib_mode = CalibrationMode::modes(m2_n_mode, 1e-6);
    let mut m2_to_closed_loop_sensor: Reconstructor =
        <WaveSensor as Calibration<GmtM2>>::calibrate(
            &closed_loop_optical_model,
            closed_loop_calib_mode.clone(),
        )?;
    m2_to_closed_loop_sensor.pseudoinverse();
    println!("{m2_to_closed_loop_sensor}");

    let mut recon =
        <DispersedFringeSensorProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate_serial(
            &optical_model,
            MirrorMode::from(CalibrationMode::RBM([
                None,                    // Tx
                None,                    // Ty
                None,                    // Tz
                Some(100f64.from_mas()), // Rx
                Some(100f64.from_mas()), // Ry
                None,                    // Rz
            ])),
            // .update((7, CalibrationMode::empty_rbm())),
            &closed_loop_optical_model,
            closed_loop_calib_mode,
        )?;
    recon.pseudoinverse();
    println!("{recon}");

    let mut data = vec![0.; 42];
    data[3] = 100f64.from_mas();
    let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
        WaveSensor,
        M1RigidBodyMotions,
    >>::estimate_with_closed_loop_reconstructor(
        &optical_model,
        &closed_loop_optical_model,
        &mut recon,
        &data,
        m2_to_closed_loop_sensor,
    )?;
    estimate
        .chunks(6)
        .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    // let mut reconc = recon.collapse();
    // reconc.pseudoinverse();
    // println!("{reconc}");

    let w =
        <DispersedFringeSensorProcessing as ClosedLoopCorrection<M1RigidBodyMotions>>::correct(
            &optical_model,
            &data,
            estimate.clone(),
            &closed_loop_optical_model,
            <DispersedFringeSensorProcessing as ClosedLoopEstimation<
                WaveSensor,
                M1RigidBodyMotions,
            >>::closed_loop_reconstructor(&mut recon),
        )?;
    dbg!(w.len());
    let n = optical_model.get_pupil_size_px();
    dbg!(n);
    Heatmap::new(n).map(&w).save("residual_wavefront.png")?;
    Ok(())
}

#[test]
fn m2_shapes() -> Result<(), Box<dyn Error>> {
    let m2_n_mode = 66;
    let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
    let optical_model = OpticalModel::<NoSensor>::builder().gmt(gmt.clone());
    let n = optical_model.get_pupil_size_px();
    dbg!(n);
    let mut om = optical_model.build()?;
    let mut data = vec![0f64; m2_n_mode * 7];
    data.chunks_mut(m2_n_mode).for_each(|data| data[0] = 1e-6);
    <OpticalModel as Read<M2ASMAsmCommand>>::read(&mut om, data.into());
    om.update();
    let w = <OpticalModel as Write<Wavefront>>::write(&mut om)
        .unwrap()
        .into_arc();
    Heatmap::new(n).map(&w).save("m2_wavefront.png")?;
    Ok(())
}

mod common;
pub use common::Heatmap;
