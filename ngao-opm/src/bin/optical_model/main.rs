/*!
Compare the wavefront using either the KL modes bases directly (from a ceo file)
or using a linear combination of ASM influence functions
```
export GMT_MODES_PATH=/home/ubuntu/projects/grsim/data/
export FEM_REPO=~/mnt/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/
cargo run --release --bin optical_model
```
*/

use std::{env, path::Path};

use crseo::{wavefrontsensor::PhaseSensor, FromBuilder, Gmt};
use gmt_dos_clients::print;
use gmt_dos_clients_crseo::OpticalModel;
use gmt_dos_clients_io::optics::{M2modes, Wavefront};
use interface::{units::MuM, Read, Size, Update, Write};
use matio_rs::MatFile;
use nalgebra as na;

const I: usize = 499;

fn main() -> anyhow::Result<()> {
    let main_rng = fastrand::Rng::with_seed(fastrand::u64(..2024));
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("bin")
        .join("optical_model");
    // M2_OrthoNormGS36p_KarhunenLoeveModes
    {
        let mut rng = main_rng.clone();
        let (m2_modes, n_mode) = ("M2_OrthoNormGS36p90_KarhunenLoeveModes", 500);
        println!("{m2_modes}");
        let mut opm = OpticalModel::<PhaseSensor>::builder()
            .gmt(
                Gmt::builder()
                    // .m1_truss_projection(false)
                    .m2(m2_modes, n_mode),
            )
            .build()?;

        let modes: Vec<_> = vec![vec![0f64; n_mode]; 7]
            .into_iter()
            .enumerate()
            .flat_map(|(i, mut modes)| {
                modes[rng.usize(..500)] = 1e-7;
                modes
            })
            .collect();

        <OpticalModel as Read<M2modes>>::read(&mut opm, modes.into());

        <OpticalModel as Update>::update(&mut opm);

        let phase = <OpticalModel as Write<MuM<Wavefront>>>::write(&mut opm).unwrap();
        let n_px = (<OpticalModel as Size<Wavefront>>::len(&mut opm) as f64).sqrt() as usize;

        let _: complot::Heatmap = (
            (phase.as_arc().as_slice(), (n_px, n_px)),
            Some(
                complot::Config::new()
                    .filename(data_path.join(format!("{m2_modes}.png")).to_str().unwrap()),
            ),
        )
            .into();
    }
    // ASMS IFs (permutated)
    {
        let mut rng = main_rng.clone();
        let (m2_modes, n_mode) = ("ASM_IFs_permutated_90", 675);
        println!("{m2_modes}");
        let mut opm = OpticalModel::<PhaseSensor>::builder()
            .gmt(
                Gmt::builder()
                    // .m1_truss_projection(false)
                    .m2(m2_modes, n_mode),
            )
            .build()?;

        let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set!");
        let fem_path = Path::new(&fem_var);

        let mat_file = MatFile::load(&fem_path.join("KLmodesGS36p90.mat"))?;
        let modes: Vec<_> = (1..=7)
            .flat_map(|i| {
                mat_file
                    .var::<_, na::DMatrix<f64>>(format!("KL_{i}"))
                    .unwrap()
                    .column(rng.usize(..500))
                    .as_slice()
                    .iter()
                    .map(|x| x * 1e-7)
                    .collect::<Vec<f64>>()
            })
            .collect();

        <OpticalModel as Read<M2modes>>::read(&mut opm, modes.into());

        <OpticalModel as Update>::update(&mut opm);

        let phase = <OpticalModel as Write<MuM<Wavefront>>>::write(&mut opm).unwrap();
        let n_px = (<OpticalModel as Size<Wavefront>>::len(&mut opm) as f64).sqrt() as usize;

        let _: complot::Heatmap = (
            (phase.as_arc().as_slice(), (n_px, n_px)),
            Some(
                complot::Config::new()
                    .filename(data_path.join(format!("{m2_modes}.png")).to_str().unwrap()),
            ),
        )
            .into();
    }
    Ok(())
}
