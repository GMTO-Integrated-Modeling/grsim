use std::env;
use std::fs::File;
use std::path::Path;

use crseo::{Builder, FromBuilder, Gmt, WavefrontSensorBuilder};
use crseo::wavefrontsensor::{LensletArray, Pyramid};

use grsim_ngao::{Processing, PyramidProcessor};

fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let n_lenslet = 92;
    let n_mode: usize = 450; //env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);
    let mut src = pym.guide_stars(None).build()?;
    let mut pym = pym.build()?;
    let m2_modes = "ASM_DDKLs_S7OC04184_675kls";
    let mut gmt = Gmt::builder().m2(m2_modes, n_mode).build()?;

    src.through(&mut gmt).xpupil().through(&mut pym);
    let pymdata = PyramidProcessor::<f32>::from(&pym).processing();

    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples").join("processing");
    serde_pickle::to_writer(&mut File::create(path.join("pymdata.pkl"))?, &pymdata, Default::default())?;

    Ok(())
}