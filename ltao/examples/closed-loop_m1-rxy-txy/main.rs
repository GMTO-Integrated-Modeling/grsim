use crseo::gmt::GmtM2;
use crseo::{FromBuilder, Gmt, Source};
use gmt_dos_clients::gif;
use gmt_dos_clients_crseo::calibration::algebra::Collapse;
use gmt_dos_clients_crseo::calibration::estimation::Estimation;
use gmt_dos_clients_crseo::calibration::{Calibration, Reconstructor};
use gmt_dos_clients_crseo::sensors::DispersedFringeSensor;
use gmt_dos_clients_crseo::{
    calibration::{
        estimation::closed_loop::ClosedLoopEstimation, CalibrationMode, ClosedLoopCalibration,
        MirrorMode,
    },
    sensors::{NoSensor, WaveSensor},
    DispersedFringeSensorProcessing, OpticalModel,
};
use gmt_dos_clients_io::gmt_m2::asm::M2ASMAsmCommand;
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    optics::{SegmentTipTilt, Wavefront},
};
use interface::{Read, Update, Write};
use skyangle::Conversion;

/*
interface_read!(OpticalModel<_>,om,M1RBM(data),...)
interface_write!(OpticalModel<_>,om,M1RBM,...)
*/

type DFS = DispersedFringeSensor<1, 1>;

fn main() -> anyhow::Result<()> {
    let m2_n_mode = 66;
    let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
    let mut om = OpticalModel::<NoSensor>::builder()
        .gmt(gmt.clone())
        .build()?;
    om.update();
    let stt0 = <OpticalModel<_> as Write<SegmentTipTilt>>::write(&mut om)
        .unwrap()
        .into_arc();
    // dbg!(&stt0);

    // Tx
    let mut data = vec![0f64; 42];
    data[0] = 1e-6;
    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut om, data.into());
    om.update();
    let stt_from_tx = <OpticalModel<_> as Write<SegmentTipTilt>>::write(&mut om)
        .map(|data| {
            data.into_arc()
                .iter()
                .zip(stt0.iter())
                .map(|(x, y)| x - y)
                .collect::<Vec<_>>()
        })
        .unwrap();
    dbg!(&stt_from_tx);
    // Ry
    let mut data = vec![0f64; 42];
    data[4] = 100f64.from_mas();
    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut om, data.into());
    om.update();
    let stt_from_ry: Vec<f64> = <OpticalModel<_> as Write<SegmentTipTilt>>::write(&mut om)
        .map(|data| {
            data.into_arc()
                .iter()
                .zip(stt0.iter())
                .map(|(x, y)| x - y)
                .collect::<Vec<_>>()
        })
        .unwrap();
    dbg!(&stt_from_ry);
    // TxRy
    let mut frame = gif::Frame::<f64>::new("m1_txry_wavefront.png", 512);
    let mut data = vec![0f64; 42];
    data[0] = 100f64.from_mas() * 2. * stt_from_ry[7] / stt_from_tx[7];
    data[4 + 3 * 6] = -100f64.from_mas();
    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut om, data.clone().into());
    om.update();
    <OpticalModel<_> as Write<Wavefront>>::write(&mut om)
        .map(|data| <gif::Frame<_> as Read<Wavefront>>::read(&mut frame, data));
    frame.update();
    frame.save()?;

    let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
    // let optical_model = OpticalModel::<DFS>::builder()
    //     .gmt(gmt.clone())
    //     .source(agws_gs.clone())
    //     .sensor(DFS::builder().source(agws_gs.clone().band("J")));
    let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());

    // let recon =
    //     <DispersedFringeSensorProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate_serial(
    //         &optical_model,
    //         MirrorMode::from(CalibrationMode::RBM([
    //             None,                    // Tx
    //             None,                    // Ty
    //             None,                    // Tz
    //             Some(100f64.from_mas()), // Rx
    //             Some(100f64.from_mas()), // Ry
    //             None,                    // Rz
    //         ]))
    //         .update((7, CalibrationMode::empty_rbm())),
    //         &closed_loop_optical_model,
    //         CalibrationMode::modes(m2_n_mode, 1e-6),
    //     )?;
    // let mut recon = recon.collapse();
    // recon.pseudoinverse();
    // println!("{recon}");

    let closed_loop_calib_mode = CalibrationMode::modes(m2_n_mode, 1e-6);
    let mut m2_to_closed_loop_sensor: Reconstructor =
        <WaveSensor as Calibration<GmtM2>>::calibrate(
            &closed_loop_optical_model,
            closed_loop_calib_mode.clone(),
        )?;
    m2_to_closed_loop_sensor.pseudoinverse();
    let m2_cmd = <WaveSensor as Estimation<M2ASMAsmCommand>>::estimate(
        &closed_loop_optical_model,
        &mut m2_to_closed_loop_sensor,
        &data,
    )
    .map(|cmd| cmd.iter().map(|x| -*x).collect::<Vec<_>>())?;
    // let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
    //     WaveSensor,
    //     M1RigidBodyMotions,
    // >>::estimate_with_closed_loop_reconstructor(
    //     &optical_model,
    //     &closed_loop_optical_model,
    //     &mut recon,
    //     &data,
    //     &mut m2_to_closed_loop_sensor,
    // )?;
    // estimate
    //     .chunks(6)
    //     .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
    //     .enumerate()
    //     .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

    let mut frame = gif::Frame::<f64>::new("m1_txry_residual_wavefront.png", 512);
    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut om, data.clone().into());
    <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(&mut om, m2_cmd.clone().into());
    om.update();
    <OpticalModel<_> as Write<Wavefront>>::write(&mut om)
        .map(|data| <gif::Frame<_> as Read<Wavefront>>::read(&mut frame, data));
    frame.update();
    frame.save()?;

    let mut frame = gif::Frame::<f64>::new("m1_txry_residual_dfs-wavefront.png", 512);
    let mut om = OpticalModel::<WaveSensor>::builder()
        .gmt(gmt.clone())
        .source(agws_gs.clone())
        .build()?;
    println!("{om}");
    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut om, data.clone().into());
    <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(&mut om, m2_cmd.clone().into());
    om.update();
    <OpticalModel<_> as Write<Wavefront>>::write(&mut om)
        .map(|data| <gif::Frame<_> as Read<Wavefront>>::read(&mut frame, data));
    frame.update();
    frame.save()?;

    Ok(())
}
