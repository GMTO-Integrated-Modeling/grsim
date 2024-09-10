use std::path::Path;

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{Average, Timer};
use gmt_dos_clients_crseo::{NoSensor, OpticalModel};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    optics::SegmentPiston,
};
use gmt_dos_clients_servos::{GmtFem, GmtServoMechanisms};
use gmt_dos_clients_windloads::system::{Mount, SigmoidCfdLoads, M1, M2};
use interface::{filing::Filing, Tick, UID};

const ACTUATOR_RATE: usize = 80;

#[derive(UID)]
pub enum AverageSegmentPiston {}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let optical_model = OpticalModel::<NoSensor>::builder().build()?;

    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path(
        Path::new(env!("FEM_REPO")).join("preloaded_servos_zen30az000_OS7.bin"),
    )?;

    let cfd_loads = Sys::<SigmoidCfdLoads>::from_path(
        Path::new(env!("FEM_REPO")).join("preloaded_windloads_zen30az000_OS7.bin"),
    )?;

    let metronome: Timer = Timer::new(8000 * 3);

    let average = Average::new(7);

    actorscript! {
    1: metronome[Tick] -> {gmt_servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}
    1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> optical_model
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> optical_model
    1: optical_model[SegmentPiston<-9>] -> average
    8000: average[AverageSegmentPiston]${7}
    }

    Ok(())
}
