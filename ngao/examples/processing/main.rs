use std::env;
use std::fs::File;
use std::path::Path;

use crseo::wavefrontsensor::{LensletArray, Pyramid};
use crseo::{Builder, FromBuilder, Gmt, WavefrontSensorBuilder};
use gmt_dos_clients_crseo::{Processing, PyramidProcessor};

fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let n_lenslet = 92;
    let n_mode: usize = 66; //env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);
    let mut src = pym.guide_stars(None).build()?;
    let mut pym = pym.build()?;
    let m2_modes = "M2_OrthoNormGS36p_KarhunenLoeveModes";
    // let m2_modes = "ASM_DDKLs_S7OC04184_675kls";
    let mut gmt = Gmt::builder().m2(m2_modes, n_mode).build()?;

    let mut a = vec![0f64; n_mode];
    a[0] = 25e-8;
    gmt.m2_segment_modes(1, &a);

    src.through(&mut gmt).xpupil().through(&mut pym);
    let pymdata = PyramidProcessor::<f32>::from(&pym).processing();

    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("processing");
    serde_pickle::to_writer(
        &mut File::create(path.join("pymdata.pkl"))?,
        &pymdata,
        Default::default(),
    )?;

    Ok(())
}
