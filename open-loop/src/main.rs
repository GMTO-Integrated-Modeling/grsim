use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Tick, Timer};
use gmt_dos_clients_crseo::{
    crseo::{atmosphere, Atmosphere, FromBuilder},
    OpticalModel,
};
use gmt_dos_clients_io::optics::WfeRms;
use std::{env, path::Path};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let sampling_frequency = 1_000f64; // Hz
    let sim_duration = 3.0;
    let n_sample = (sim_duration * sampling_frequency) as usize;

    // Atmospher builder
    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(sim_duration)
            .filepath(&data_repo.join("atmosphere.bin")),
    );

    // GMT optical model
    let gom = OpticalModel::builder()
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let heartbeat: Timer = Timer::new(n_sample);

    actorscript! {
        #[model(state = completed)]
        1: heartbeat[Tick] -> gom[WfeRms]$
    }

    Ok(())
}
