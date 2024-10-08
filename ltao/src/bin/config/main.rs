use std::{fs::File, path::Path};

use crseo::{imaging::LensletArray, FromBuilder, Gmt, Source};
use gmt_dos_clients_crseo::{
    sensors::{Camera, WaveSensor},
    Centroids, OpticalModel,
};
use skyangle::Conversion;

const M1_N_MODE: usize = 27;
const M2_N_MODE: usize = 66;
const AGWS_N_GS: usize = 3;

fn main() -> anyhow::Result<()> {
    let path = Path::new("src/bin/merged_reconstructor");
    let gmt_builder = Gmt::builder()
        .m1("bending modes", M1_N_MODE)
        .m2("Karhunen-Loeve", M2_N_MODE)
        .m1_truss_projection(false);

    let agws_gs_builder = Source::builder()
        .size(AGWS_N_GS)
        .on_ring(6f32.from_arcmin());

    // AGWS SH48 ...
    let sh48 = Camera::builder()
        .n_sensor(AGWS_N_GS)
        .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32))
        .lenslet_flux(0.75);
    let mut sh48_centroids: Centroids = Centroids::try_from(&sh48)?;

    let sh48_om_builder = OpticalModel::<Camera<1>>::builder()
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone())
        .sensor(sh48);

    
    Ok(())
}
