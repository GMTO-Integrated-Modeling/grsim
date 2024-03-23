use std::{env, path::Path};

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::Timer;
use gmt_dos_clients_io::cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads};
use gmt_dos_clients_servos::{
    asms_servo::ReferenceBody, AsmsServo, GmtFem, GmtServoMechanisms, WindLoads,
};
use gmt_dos_clients_windloads::{
    system::{Mount, SigmoidCfdLoads, M1, M2},
    CfdLoads,
};
use gmt_fem::FEM;
use interface::{filing::Filing, Tick};

const ACTUATOR_RATE: usize = 80;

const PRELOADING_N_SAMPLE: usize = 8000 * 3;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sim_sampling_frequency = 8000;
    let sim_duration = 30_usize; // second

    let (cfd_loads, gmt_servos) = {
        let mut fem = FEM::from_env()?;
        // The CFD wind loads must be called next afer the FEM as it is modifying
        // the FEM CFDMountWindLoads inputs
        let cfd_loads = Sys::<SigmoidCfdLoads>::try_from(
            CfdLoads::foh(".", sim_sampling_frequency)
                .duration(sim_duration as f64)
                .mount(&mut fem, 0, None)
                .m1_segments()
                .m2_segments(),
        )?;

        let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::try_from(
            GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
                .wind_loads(WindLoads::new())
                .asms_servo(
                    AsmsServo::new()
                        .facesheet(Default::default())
                        .reference_body(ReferenceBody::new()),
                ),
        )?;

        (cfd_loads, gmt_servos)
    };

    let metronome: Timer = Timer::new(PRELOADING_N_SAMPLE);

    actorscript! {
    1: metronome[Tick] -> {gmt_servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}
    }
    let path = Path::new(env!("FEM_REPO"));
    gmt_servos.to_path(path.join("preloaded_servos_zen30az000_OS7_1.bin"))?;
    cfd_loads.to_path(path.join("preloaded_windloads_zen30az000_OS7_1.bin"))?;

    Ok(())
}
