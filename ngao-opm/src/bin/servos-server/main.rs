use std::{env, path::Path};

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::{M2ASMAsmCommand, M2ASMFaceSheetFigure, M2ASMReferenceBodyNodes},
};
use gmt_dos_clients_servos::{GmtFem, GmtM2, GmtServoMechanisms};
use interface::filing::Filing;

const ACTUATOR_RATE: usize = 80;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    if cfg!(feature = "windloads") {
        preloaded_with_windloads_fem().await?;
    } else {
        fem().await?;
    }
    Ok(())
}

async fn fem() -> anyhow::Result<()> {
    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path(
        Path::new(env!("FEM_REPO")).join("servos_1.bin"),
    )?;

    actorscript! {
        #[model(name=servos_server)]
        1: >>[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure]>>
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions]>>
        1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes]>>
    }

    Ok(())
}

async fn preloaded_with_windloads_fem() -> anyhow::Result<()> {
    use gmt_dos_clients_windloads::system::{Mount, SigmoidCfdLoads, M1, M2};
    let path = Path::new(env!("FEM_REPO"));
    let cfd_loads =
        Sys::<SigmoidCfdLoads>::from_path(path.join("preloaded_windloads_zen30az000_OS7_1.bin"))?;

    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path(
        path.join("preloaded_servos_zen30az000_OS7_1.bin"),
    )?;

    actorscript! {
        #[model(name=preloaded_servos_server)]
        1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
        1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
        1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}

        1: >>[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure]>>
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions]>>
        1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes]>>
    }

    Ok(())
}
