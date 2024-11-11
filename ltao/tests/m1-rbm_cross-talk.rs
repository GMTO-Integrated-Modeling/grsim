use std::error::Error;

use crseo::gmt::GmtM1;
use gmt_dos_clients_crseo::{
    calibration::{estimation::Estimation, Calibrate, CalibrationMode},
    sensors::WaveSensor,
    OpticalModel,
};
use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
use skyangle::Conversion;

#[test]
fn main() -> Result<(), Box<dyn Error>> {
    let optical_model = OpticalModel::<WaveSensor>::builder();

    let mut recon = <WaveSensor as Calibrate<GmtM1>>::calibrate(
        &optical_model,
        CalibrationMode::RBM([
            None,
            None,
            Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            None,
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    for (i, rbm) in ["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"].into_iter().enumerate() {
        println!("Cross-talk for {rbm}");
        let mut data = vec![0f64; 42];
        data.chunks_mut(6).for_each(|x| x[i] = 1e-6);
        let estimate = <WaveSensor as Estimation<M1RigidBodyMotions>>::estimate(
            &optical_model,
            &mut recon,
            data,
        )?;
        println!("S#: [Txyz[mum]] , [Rxyz[mas]]");
        estimate.chunks(6).enumerate().for_each(|(i, e)| {
            println!(
                "S{}: {:+6.0?} {:+6.0?}",
                i + 1,
                e[..3].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
                e[3..].iter().map(|x| x.to_mas()).collect::<Vec<_>>()
            )
        });
    }
    Ok(())
}
