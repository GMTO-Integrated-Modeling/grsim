use crseo::{
    centroiding::{CentroidingBuilder},
    imaging::{ImagingBuilder, LensletArray},
    Builder, FromBuilder, Gmt, Source,
};
use gmt_dos_clients_crseo::{
    centroiding::CentroidsProcessing, sensors::Camera, DeviceInitialize, OpticalModel,
};

fn main() -> anyhow::Result<()> {
    let n_gs = 1;
    let m1_n_mode = 6;
    let m2_n_mode = 15;

    {
        let mut gmt = Gmt::builder().build()?;
        let mut src = Source::builder()
            .size(n_gs)
            .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs])
            .pupil_sampling(48 * 32 + 1)
            .build()?;
        println!("{src}");

        let imgr_builder = ImagingBuilder::default()
            .n_sensor(n_gs)
            .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32));

        let mut centroiding = CentroidingBuilder::from(&imgr_builder).build()?;

        let mut imgr = imgr_builder.build()?;
        println!("{imgr}");

        src.through(&mut gmt).xpupil().through(&mut imgr);

        let frame = imgr.frame();
        dbg!(Vec::<f32>::from(&frame).iter().sum::<f32>());
        centroiding.process(&frame, None);

        dbg!(centroiding.integrated_flux());
    }

    let agws_gs = Source::builder()
        .size(n_gs)
        .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs]); //.on_ring(6f32.from_arcmin());
    let sh48 = Camera::builder()
        .n_sensor(n_gs)
        .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32));
    // .lenslet_flux(0.75);
    let mut sh48_centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sh48)?;

    let gmt = Gmt::builder()
        .m1("bending modes", m1_n_mode)
        .m2("Karhunen-Loeve", m2_n_mode);

    let optical_model = OpticalModel::<Camera<1>>::builder()
        .gmt(gmt.clone())
        .source(agws_gs.clone())
        .sensor(sh48);

    optical_model.initialize(&mut sh48_centroids);
    dbg!(sh48_centroids.n_valid_lenslets());

    let om = optical_model.build()?;
    println!("{om}");
    Ok(())
}

/*

[/home/ubuntu/data/home/ubuntu/projects/crseo/src/centroiding.rs:193:42] flux.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() = 32675584.0
[/home/ubuntu/data/home/ubuntu/projects/crseo/src/centroiding.rs:193:42] flux.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() = 158938450.0
[/home/ubuntu/data/home/ubuntu/projects/crseo/src/centroiding.rs:193:42] flux.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() =
  178 435 380.0
2 556 058 400.0
*/
