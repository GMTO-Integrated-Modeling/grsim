use std::{fs::File, path::Path};

use crseo::gmt::GmtM2;
use faer::mat::from_column_major_slice;
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif, print::Print, Signals};
use gmt_dos_clients_crseo::{
    calibration::{estimation::Estimation, Calibration, CalibrationMode, Reconstructor},
    sensors::WaveSensor,
    OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{Dev, Frame, M2GlobalTipTilt, SensorData, Wavefront, WfeRms},
};
use ltao_tests::{M2GttToPtt, Model, Models};
use skyangle::Conversion;

const MIN_N_MODE: usize = 3;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let models = Models::new();
    let om =
        OpticalModel::<WaveSensor>::builder().gmt(models.gmt_builder.clone().m2_n_mode(MIN_N_MODE));

    // M2 1,2,3 modes calibration
    let mut m2_modes: Reconstructor = <WaveSensor as Calibration<GmtM2>>::calibrate(
        &om,
        CalibrationMode::modes(MIN_N_MODE, 1e-8),
    )?;
    m2_modes.pseudoinverse();
    println!("{m2_modes}");

    // Mapping of M2 global tip-tilt with M1 1,2,3 modes
    let s = 250f64.from_mas();
    let m2_cmd_x: Vec<_> =
        <WaveSensor as Estimation<M2GlobalTipTilt>>::estimate(&om, &mut m2_modes, &vec![s, 0f64])?
            .iter()
            .zip(
                <WaveSensor as Estimation<M2GlobalTipTilt>>::estimate(
                    &om,
                    &mut m2_modes,
                    &vec![-s, 0f64],
                )?
                .iter(),
            )
            .map(|(x, y)| 0.5 * (x - y) / s)
            .collect();
    let m2_cmd_y: Vec<_> =
        <WaveSensor as Estimation<M2GlobalTipTilt>>::estimate(&om, &mut m2_modes, &vec![0f64, s])?
            .iter()
            .zip(
                <WaveSensor as Estimation<M2GlobalTipTilt>>::estimate(
                    &om,
                    &mut m2_modes,
                    &vec![0f64, -s],
                )?
                .iter(),
            )
            .map(|(x, y)| 0.5 * (x - y) / s)
            .collect();
    let m2_cmd = m2_cmd_x
        .into_iter()
        .chain(m2_cmd_y.into_iter())
        .collect::<Vec<_>>();
    let m2_gtt_to_modes = from_column_major_slice::<f64>(&m2_cmd, MIN_N_MODE * 7, 2);
    dbg!(m2_gtt_to_modes.shape());

    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("m2_gtt-to-modes");
    {
        // Applied M2 global tip-tilt using M2 RBM
        let models = Models::new();
        let oiwfs = models.oiwfs().build()?;
        println!("{oiwfs}");
        let oiwfs_centroids = models.oiwfs().processor()?;
        let signal = Signals::new(2, 1).channel(0, 250f64.from_mas());
        let print = Print::new(1);

        let frame: gif::Frame<f64> = gif::Frame::new(data_path.join("opd_gtt.png"), 512);

        actorscript!(
        1: signal[M2GlobalTipTilt]
            -> oiwfs[Frame<Dev>]
                -> oiwfs_centroids[SensorData]
                    -> print
        1: oiwfs[Wavefront] -> frame
           );
        let _ = frame.lock().await.save()?;
    }
    {
        // Applied M2 global tip-tilt using M2 1,2,3 modes
        let models = Models::new();
        let oiwfs = models
            .oiwfs()
            .builder()
            .gmt(models.gmt_builder.clone().m2_n_mode(MIN_N_MODE))
            .build()?;
        let oiwfs_centroids = models.oiwfs().processor()?;
        let signal = Signals::new(2, 1).channel(0, 250f64.from_mas());
        let print = Print::new(1);
        let gain = M2GttToPtt::new(m2_gtt_to_modes.to_owned());

        serde_pickle::to_writer(
            &mut File::create(data_path.join("m2_gtt_to_ptt.pkl"))?,
            &gain,
            Default::default(),
        )?;

        let frame: gif::Frame<f64> = gif::Frame::new(data_path.join("opd_stt.png"), 512);

        actorscript!(
        1: signal[M2GlobalTipTilt]
              -> gain[M2ASMAsmCommand]
                  -> oiwfs[Frame<Dev>]
                      -> oiwfs_centroids[SensorData]
                        -> print
        1: oiwfs[Wavefront] -> frame
        );
        let _ = frame.lock().await.save()?;
    }
    {
        // Applied M2 global tip-tilt using M2 1,2,3 modes
        let models = Models::new();
        let oiwfs = models
            .oiwfs()
            .builder()
            .gmt(models.gmt_builder.clone().m2_n_mode(MIN_N_MODE))
            .build()?;
        let oiwfs_centroids = models.oiwfs().processor()?;
        let signal0 = Signals::new(2, 1).channel(0, -250f64.from_mas());
        let signal = Signals::new(2, 1).channel(0, 250f64.from_mas());
        let print = Print::new(1);
        let gain = M2GttToPtt::new(m2_gtt_to_modes.to_owned());

        let frame: gif::Frame<f64> = gif::Frame::new(data_path.join("opd_res.png"), 512);

        actorscript!(
        1: signal0[M2GlobalTipTilt] -> oiwfs
        1: signal[M2GlobalTipTilt]
              -> gain[M2ASMAsmCommand]
                  -> oiwfs[Frame<Dev>]
                      -> oiwfs_centroids[SensorData]
                        -> print
        1: oiwfs[Wavefront] -> frame
        1: oiwfs[WfeRms<-9>] -> print
        );
        let _ = frame.lock().await.save()?;
    }
    Ok(())
}
