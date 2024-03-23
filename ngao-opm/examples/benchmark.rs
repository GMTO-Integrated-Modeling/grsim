use std::{env, path::Path};

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::Timer;
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
};
use gmt_dos_clients_servos::{GmtFem, GmtServoMechanisms};
use gmt_dos_clients_windloads::system::{Mount, SigmoidCfdLoads, M1, M2};
use interface::{filing::Filing, Tick};

const ACTUATOR_RATE: usize = 80;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let path = Path::new(env!("FEM_REPO"));
    let cfd_loads =
        Sys::<SigmoidCfdLoads>::from_path(path.join("preloaded_windloads_zen30az000_OS7_1.bin"))?;

    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path(
        path.join("preloaded_servos_zen30az000_OS7_1.bin"),
    )?;

    let metronome: Timer = Timer::new(400);

    actorscript! {
    1: metronome[Tick] -> {gmt_servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}

    1: {gmt_servos::GmtFem}[M1RigidBodyMotions]$
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions]$
    }

    Ok(())
}
