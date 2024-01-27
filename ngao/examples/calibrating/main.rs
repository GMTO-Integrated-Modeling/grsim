use std::env;
use std::fs::File;
use std::path::Path;
use std::sync::Arc;

use crseo::wavefrontsensor::{LensletArray, Pyramid};
use crseo::{Builder, FromBuilder, Gmt, WavefrontSensorBuilder};

use gmt_dos_clients_crseo::{
    Calibration, Processor, PyramidCalibrator, PyramidMeasurements, PyramidProcessor,
};
use gmt_dos_clients_crseo::{
    DetectorFrame, GuideStar, OpticalModel, ResidualM2modes, WavefrontSensor,
};
use gmt_dos_clients_io::optics::M2modes;
use interface::{Data, Read, UniqueIdentifier, Update, Write};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let n_lenslet = 92;
    // let (m2_modes, n_mode) = ("ASM_DDKLs_S7OC04184_675kls", 500);
    let (m2_modes, n_mode) = ("M2_OrthoNormGS36p_KarhunenLoeveModes", 500);

    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);

    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("calibrating");
    let file_name = format!("pymtor_{m2_modes}_{n_mode}.pkl");
    let mut pymtor: PyramidCalibrator = if !path.join(&file_name).exists() {
        let pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, n_mode)
            // .stroke(25e-10)
            // .piston_mask_threshold(0.65)
            .n_gpu(8)
            .build()?;
        serde_pickle::to_writer(
            &mut File::create(path.join(&file_name))?,
            &pymtor,
            Default::default(),
        )?;
        pymtor
    } else {
        serde_pickle::from_reader(File::open(path.join(&file_name))?, Default::default())?
    };

    let sampling_frequency = 100usize; // Hz

    let mut gom = OpticalModel::builder()
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .source(pym.guide_stars(None))
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let pym = pym.build()?;
    let mut processor: Processor<_> = Processor::from(&pym);
    let mut sensor = WavefrontSensor::<_, 1>::new(pym);

    dbg!(pymtor.h_matrix_cond());
    pymtor.h0_estimator()?;
    println!("{pymtor}");
    let mut calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    let mut data = vec![0f64; n_mode * 7];
    data[1] = 25e-9;
    data[n_mode * 7 - 1] = -25e-9;
    model(
        data.clone(),
        n_mode,
        &mut gom,
        &mut sensor,
        &mut processor,
        &mut calibrator,
    );
    model(
        data,
        n_mode,
        &mut gom,
        &mut sensor,
        &mut processor,
        &mut calibrator,
    );

    dbg!(pymtor.p_matrix_cond());
    pymtor.hp_estimator()?;
    println!("{pymtor}");
    let mut calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    let mut data = vec![0f64; n_mode * 7];
    data[0] = 25e-9;
    data[n_mode * 5] = -25e-9;
    model(
        data.clone(),
        n_mode,
        &mut gom,
        &mut sensor,
        &mut processor,
        &mut calibrator,
    );
    data.fill(0f64);
    data[n_mode * 6] = 25e-9;
    model(
        data,
        n_mode,
        &mut gom,
        &mut sensor,
        &mut processor,
        &mut calibrator,
    );

    let pym_constrained_recon: nalgebra::DMatrix<f32> = serde_pickle::from_reader(
        // File::open(path.join("pym_constrained_recon.pkl"))?,
        File::open(path.join("pym_M2_OrthoNormGS36_KarhunenLoeveModes_496_constrained_recon.pkl"))?,
        Default::default(),
    )?;
    dbg!(pym_constrained_recon.shape());
    pymtor.set_hp_estimator(pym_constrained_recon);
    println!("{pymtor}");
    let mut calibrator: Calibration<PyramidCalibrator> = pymtor.into();
    let mut data = vec![0f64; n_mode * 7];
    data[0] = 25e-9;
    data[n_mode * 5] = -25e-9;
    model(
        data.clone(),
        n_mode,
        &mut gom,
        &mut sensor,
        &mut processor,
        &mut calibrator,
    );
    data.fill(0f64);
    data[n_mode * 6] = 25e-9;
    model(
        data,
        n_mode,
        &mut gom,
        &mut sensor,
        &mut processor,
        &mut calibrator,
    );

    Ok(())
}

fn model(
    input: Vec<f64>,
    n_mode: usize,
    gom: &mut OpticalModel,
    sensor: &mut WavefrontSensor<Pyramid, 1>,
    processor: &mut Processor<PyramidProcessor<f32>>,
    calibrator: &mut Calibration<PyramidCalibrator>,
) {
    <OpticalModel as Read<M2modes>>::read(gom, Data::new(input));
    gom.update();
    let data = <OpticalModel as Write<GuideStar>>::write(gom).unwrap();
    <WavefrontSensor<Pyramid> as Read<GuideStar>>::read(sensor, data);
    let data = <WavefrontSensor<Pyramid> as Write<DetectorFrame<f32>>>::write(sensor).unwrap();
    <Processor<PyramidProcessor<f32>> as Read<DetectorFrame<f32>>>::read(processor, data);
    let data =
        <Processor<PyramidProcessor<f32>> as Write<PyramidMeasurements>>::write(processor).unwrap();
    <Calibration<PyramidCalibrator> as Read<PyramidMeasurements>>::read(calibrator, data);
    <Calibration<PyramidCalibrator> as Write<ResidualM2modes>>::write(calibrator)
        .unwrap()
        .chunks(n_mode)
        .map(|x| x.iter().take(15).map(|x| x * 1e9).collect::<Vec<_>>())
        .for_each(|c| println!("{c:+3.0?}"));
    println!("");
}

#[derive(Debug, Default)]
pub struct Print<T> {
    counter: usize,
    data: Option<Arc<Vec<T>>>,
}

impl<T> Update for Print<T>
where
    T: Send + Sync + std::fmt::Debug,
{
    fn update(&mut self) {
        if let Some(data) = self.data.as_ref() {
            println!(" #{:>5}: {:4.2?}", self.counter, data);
            self.counter += 1;
        }
    }
}

impl<T, U> Read<U> for Print<T>
where
    T: Send + Sync + std::fmt::Debug,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        self.data = Some(data.as_arc());
    }
}
