use crseo::{
    gmt::{GmtM1, GmtM2},
    FromBuilder, Gmt, Source,
};
use gmt_dos_clients_crseo::{
    calibration::{Calibration, CalibrationMode},
    sensors::{builders::WaveSensorBuilder, WaveSensor},
    OpticalModel,
};
use skyangle::Conversion;

const M1_N_MODE: usize = 27;
const M2_N_MODE: usize = 66;

fn main() -> anyhow::Result<()> {
    let calib_m2_modes_onaxis = {
        // On-axis calibration of M2 RBM
        println!("On-axis calibration of M2 RBM");
        let rec = <WaveSensor as Calibration<GmtM2>>::calibrate(
            &OpticalModel::<WaveSensor>::builder()
                .gmt(Gmt::builder().m2("Karhunen-Loeve", M2_N_MODE))
                .sensor(WaveSensorBuilder::default()),
            CalibrationMode::modes(M2_N_MODE, 1e-6),
        )
        .unwrap();
        // rec.pseudoinverse();
        rec
    };
    println!("{calib_m2_modes_onaxis}");

    let srcs = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let calib_m2_modes_offaxis = {
        // Off-axis calibration of M2 modes
        println!("Off-axis calibration of M2 modes");
        let rec = <WaveSensor as Calibration<GmtM2>>::calibrate(
            &OpticalModel::<WaveSensor>::builder()
                .gmt(Gmt::builder().m2("Karhunen-Loeve", M2_N_MODE))
                .source(srcs.clone())
                .sensor(WaveSensorBuilder::default().source(srcs)),
            CalibrationMode::modes(M2_N_MODE, 1e-6),
        )
        .unwrap();
        // rec.pseudoinverse();
        rec
    };
    println!("{calib_m2_modes_offaxis}");

    // On-axis calibration of M1 RBM
    let calib_m1_rbm_onaxis = {
        println!("On-axis calibration of M1 RBM");
        let rec = <WaveSensor as Calibration<GmtM1>>::calibrate(
            &OpticalModel::<WaveSensor>::builder().sensor(WaveSensorBuilder::default()),
            CalibrationMode::RBM([
                None,                     // Tx
                None,                     // Ty
                None,                     // Tz
                Some(1f64.from_arcsec()), // Rx
                Some(1f64.from_arcsec()), // Ry
                None,                     // Rz
            ]),
        )
        .unwrap();
        // rec.pseudoinverse();
        rec
    };
    println!("{calib_m1_rbm_onaxis}");

    // Off-axis calibration of M1 RBM
    let srcs = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let calib_m1_rbm_offaxis = {
        println!("Off-axis calibration of M1 RBM");
        let rec = <WaveSensor as Calibration<GmtM1>>::calibrate(
            &OpticalModel::<WaveSensor>::builder()
                .source(srcs.clone())
                .sensor(WaveSensorBuilder::default().source(srcs)),
            CalibrationMode::RBM([
                None,                     // Tx
                None,                     // Ty
                None,                     // Tz
                Some(1f64.from_arcsec()), // Rx
                Some(1f64.from_arcsec()), // Ry
                None,                     // Rz
            ]),
        )
        .unwrap();
        // rec.pseudoinverse();
        rec
    };
    println!("{calib_m1_rbm_offaxis}");

    serde_pickle::to_writer(
        &mut std::fs::File::create("src/bin/calibration/calib_m2_modes_onaxis.pkl")?,
        &calib_m2_modes_onaxis,
        Default::default(),
    )?;
    serde_pickle::to_writer(
        &mut std::fs::File::create("src/bin/calibration/calib_m2_modes_offaxis.pkl")?,
        &calib_m2_modes_offaxis,
        Default::default(),
    )?;
    serde_pickle::to_writer(
        &mut std::fs::File::create("src/bin/calibration/calib_m1_rbm_onaxis.pkl")?,
        &calib_m1_rbm_onaxis,
        Default::default(),
    )?;
    serde_pickle::to_writer(
        &mut std::fs::File::create("src/bin/calibration/calib_m1_rbm_offaxis.pkl")?,
        &calib_m1_rbm_offaxis,
        Default::default(),
    )?;

    Ok(())
}
