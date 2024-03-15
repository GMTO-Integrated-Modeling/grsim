use crseo::{
    wavefrontsensor::{LensletArray, Pyramid},
    FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, OpticalModel, Processor, PyramidCalibrator, PyramidMeasurements,
    PyramidProcessor, ResidualM2modes,
};
use gmt_dos_clients_io::optics::{M2modes, SegmentPiston};
use interface::{Data, Read, Update, Write};

#[test]
fn piston() -> anyhow::Result<()> {
    let n_lenslet = 92;
    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);
    // let (m2_modes, n_mode) = ("ASM_DDKLs_S7OC04184_675kls", 500);
    let (m2_modes, n_mode) = ("M2_OrthoNormGS36p90_KarhunenLoeveModes", 500);
    let mut optical_model = OpticalModel::<Pyramid>::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, n_mode),
        )
        .source(pym.guide_stars(None))
        .sensor(pym.clone())
        .build()?;

    let stroke = 1e-6;
    println!("Piston scaling factor");
    let data: Vec<_> = vec![vec![0f64; n_mode]; 7]
        .into_iter()
        .flat_map(|mut modes| {
            modes[0] = stroke;
            modes
        })
        .collect();
    <OpticalModel<Pyramid> as Read<M2modes>>::read(&mut optical_model, Data::new(data));
    optical_model.update();
    let piston_scale: Vec<_> =
        <OpticalModel<Pyramid> as Write<SegmentPiston>>::write(&mut optical_model)
            .map(|sp| {
                (*sp)
                    .iter()
                    .map(|sp| (sp / stroke).recip())
                    .inspect(|x| println!("{}", x))
                    .collect()
            })
            .unwrap();

    println!("X-pupil piston with scaling");
    let data: Vec<_> = vec![vec![0f64; n_mode]; 7]
        .into_iter()
        .zip(&piston_scale)
        .flat_map(|(mut modes, ps)| {
            modes[0] = stroke * ps;
            modes
        })
        .collect();
    <OpticalModel<Pyramid> as Read<M2modes>>::read(&mut optical_model, Data::new(data));
    optical_model.update();
    <OpticalModel<Pyramid> as Write<SegmentPiston>>::write(&mut optical_model)
        .map(|sp| (*sp).iter().for_each(|x| println!("{}", x / stroke)));

    Ok(())
}
#[test]
fn reconstructor() -> anyhow::Result<()> {
    let n_lenslet = 92;
    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);
    let (m2_modes, n_mode) = ("ASM_DDKLs_S7OC04184_675kls", 500);
    // let (m2_modes, n_mode) = ("M2_OrthoNormGS36p90_KarhunenLoeveModes", 500);
    let mut optical_model = OpticalModel::<Pyramid>::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, n_mode),
        )
        .source(pym.guide_stars(None))
        .sensor(pym.clone())
        .build()?;
    let mut processor: Processor<_> = Processor::try_from(&pym)?;

    let filename = format!("{0}/pym-{m2_modes}-{n_mode}.bin", env!("GMT_MODES_PATH"));
    let mut pymtor = PyramidCalibrator::try_from(filename.as_str())?;
    pymtor.p_estimator()?;
    println!("{pymtor}");
    let mut calibrator: Calibration<PyramidCalibrator> = pymtor.into();

    let mut data = vec![0f64; n_mode * 7];
    data[0] = 50e-9;

    <OpticalModel<Pyramid> as Read<M2modes>>::read(&mut optical_model, Data::new(data));
    optical_model.update();
    <OpticalModel<Pyramid> as Write<DetectorFrame>>::write(&mut optical_model).map(|data| {
        <Processor<PyramidProcessor> as Read<DetectorFrame>>::read(&mut processor, data)
    });
    <Processor<PyramidProcessor> as Write<PyramidMeasurements>>::write(&mut processor).map(
        |data| {
            <Calibration<PyramidCalibrator> as Read<PyramidMeasurements>>::read(
                &mut calibrator,
                data,
            )
        },
    );
    <Calibration<PyramidCalibrator> as Write<ResidualM2modes>>::write(&mut calibrator)
        .map(|data| dbg!(data[0]));
    Ok(())
}
