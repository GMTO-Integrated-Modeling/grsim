use std::{env, path::Path};

use crseo::{Atmosphere, FromBuilder, RayTracing};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::Timer;
use gmt_dos_clients_crseo::{sensors::NoSensor, OpticalModel};
use gmt_dos_clients_io::optics::{SegmentPiston, SegmentWfeRms, WfeRms};
use interface::Tick;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("atmosphere");
    env::set_var("DATA_REPO", &data_path);

    let sampling_frequency = 500f64;
    // let atm_builder = Atmosphere::builder().ray_tracing(
    //     RayTracing::default()
    //         .field_size(10f64.from_arcmin())
    //         .duration(60.)
    //         .n_duration(15)
    //         .filepath("ltao-atmosphere.bin"),
    // );
    let atm_builder = Atmosphere::builder()
        .single_turbulence_layer(0f32, Some(7f32), Some(0f32))
        .ray_tracing(
            RayTracing::default()
                .duration(5.)
                .n_duration(100)
                .filepath("atm_single_layer.bin"),
        );

    let om = OpticalModel::<NoSensor>::builder()
        .sampling_frequency(sampling_frequency)
        .atmosphere(atm_builder)
        .build()?;

    let timer: Timer = Timer::new(25_000);
    actorscript!(
        #[model(name=atmosphere)]
        1: timer[Tick] -> om
        1: om[WfeRms<-9>]$
        1: om[SegmentWfeRms<-9>]$
        1: om[SegmentPiston<-9>]$
    );

    Ok(())
}
