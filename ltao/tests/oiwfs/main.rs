use std::error::Error;

use crseo::{gmt::GmtM2, imaging::Detector, FromBuilder, Gmt, Source};
use gmt_dos_clients_crseo::{
    calibration::{estimation::Estimation, Calibrate, CalibrationMode, Reconstructor},
    centroiding::CentroidsProcessing,
    sensors::Camera,
    OpticalModel,
};
use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
use skyangle::Conversion;

const M1_N_MODE: usize = 27;
const M2_N_MODE: usize = 66;

#[test]
fn main() -> Result<(), Box<dyn Error>> {
    let gmt_builder = Gmt::builder()
        .m1("bending modes", M1_N_MODE)
        .m2("Karhunen-Loeve", M2_N_MODE)
        .m1_truss_projection(false);

    let oiwfs = Camera::builder().detector(Detector::default().n_px_imagelet(512));

    let optical_model = OpticalModel::<Camera<1>>::builder()
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("K"))
        .sensor(oiwfs);

    let mut recon: Reconstructor = <CentroidsProcessing as Calibrate<GmtM2>>::calibrate(
        &((&optical_model).into()),
        CalibrationMode::modes(M2_N_MODE, 1e-8) // .start_from(2)
            .ends_at(3),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    let mut data = vec![0.; 42];
    data[3] = 100f64.from_mas();
    let estimate = <CentroidsProcessing as Estimation<M1RigidBodyMotions>>::estimate(
        &optical_model,
        &mut recon,
        data,
    )?;
    dbg!(estimate.len());
    estimate
        .chunks(M2_N_MODE)
        .map(|c| c.iter().take(6).map(|x| x * 1e9).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("S{}: {:6.0?}", i + 1, x));

    Ok(())
}
