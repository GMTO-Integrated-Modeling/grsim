use std::{env, path::Path};

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::{M2ASMAsmCommand, M2ASMFaceSheetFigure, M2ASMReferenceBodyNodes},
};
use gmt_dos_clients_servos::{
    asms_servo::ReferenceBody, AsmsServo, GmtFem, GmtM2, GmtServoMechanisms,
};

use gmt_dos_clients_transceiver::{Monitor, Transceiver};
use gmt_dos_clients_windloads::system::{Mount, SigmoidCfdLoads, M1, M2};
use gmt_fem::FEM;
use interface::{filing::Filing, Tick};

const ACTUATOR_RATE: usize = 80;

const PRELOADING_N_SAMPLE: usize = 8000 * 3;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sim_sampling_frequency = 8000;
    let sim_duration = 30_usize; // second

    let server_address = env::var("SERVOS_SERVER_IP")?;
    let client_address = env::var("SERVOS_CLIENT_IP")?;
    let mut monitor = Monitor::new();

    let m2_asm_facesheet =
        Transceiver::<M2ASMFaceSheetFigure>::transmitter(&server_address)?.run(&mut monitor);
    let m1_rbms =
        Transceiver::<M1RigidBodyMotions>::transmitter(&server_address)?.run(&mut monitor);
    let m2_rbms =
        Transceiver::<M2ASMReferenceBodyNodes>::transmitter(&server_address)?.run(&mut monitor);

    let rx_address = "0.0.0.0:0";
    let m2_asm_cmd =
        Transceiver::<M2ASMAsmCommand>::receiver(&client_address, rx_address)?.run(&mut monitor);

    /*     let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path(
        Path::new(env!("FEM_REPO")).join("servos.bin"),
    )?;


    actorscript! {
        1: m2_asm_cmd[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> m2_asm_facesheet
    } */

    let path = Path::new(env!("FEM_REPO"));
    let cfd_loads =
        Sys::<SigmoidCfdLoads>::from_path(path.join("preloaded_windloads_zen30az000_OS7_1.bin"))?;

    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path(
        path.join("preloaded_servos_zen30az000_OS7_1.bin"),
    )?;

    actorscript! {
        1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
        1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
        1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}

        1: m2_asm_cmd[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> m2_asm_facesheet
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> m1_rbms
        1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes] -> m2_rbms

        // 1: {gmt_servos::GmtFem}[M1RigidBodyMotions]$
        // 1: {gmt_servos::GmtFem}[M2RigidBodyMotions]$
    }

    monitor.await?;

    Ok(())
}
