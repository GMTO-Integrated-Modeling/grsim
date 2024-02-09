use std::{env, path::Path};

use crseo::{
    atmosphere,
    Atmosphere, Gmt, FromBuilder,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::Weight;
use gmt_dos_clients::{OneSignal, Signal, Signals, Smooth};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::{M1RigidBodyMotions},
    gmt_m2::{
        //asm::{M2ASMAsmCommand, M2ASMReferenceBodyNodes},
        M2RigidBodyMotions,
    },
    //mount::MountSetPoint,
    optics::{WfeRms,SegmentWfeRms},
};
use gmt_dos_clients_servos::*;
use gmt_dos_clients_crseo::{OpticalModel, GuideStar, WavefrontStats};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use gmt_dos_clients_windloads::CfdLoads;
use gmt_fem::FEM;


const ACTUATOR_RATE: usize = 80;

/*
FEM_REPO=`pwd`/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/ cargo run --release --bin windloaded-servos
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let sim_sampling_frequency = 8000;
    let sim_duration = 5_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = FEM::from_env()?;
    // M2 modal basis
    let (m2_modes, n_mode) = ("M2_OrthoNormGS36p_KarhunenLoeveModes", 500);

    // Optical Model (GMT with 1 guide star on-axis)
    // Atmospher builder
    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(5f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );
    let optical_model = OpticalModel::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, n_mode),
        )
        //.source(pym.guide_stars(None))
        .atmosphere(atm_builder)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;

    // The CFD wind loads must be called next afer the FEM as it is modifying
    // the FEM CFDMountWindLoads inputs
    let cfd_loads = CfdLoads::foh(".", sim_sampling_frequency)
        .duration(sim_duration as f64)
        .mount(&mut fem, 0, None)
        .m1_segments()
        .m2_segments()
        .build()?;

    // MOUNT SET POINT
    // let setpoint = Signals::new(3, n_step); //.channel(1, Signal::Constant(1f64.from_arcsec()));

    // ...
    let stats = WavefrontStats::<1>::default();

    let sigmoid = OneSignal::try_from(Signals::new(1, n_step).channel(
        0,
        Signal::Sigmoid {
            amplitude: 1f64,
            sampling_frequency_hz: sim_sampling_frequency as f64,
        },
    ))?;

    let m1_smoother = Smooth::new();
    let m2_smoother = Smooth::new();
    let mount_smoother = Smooth::new();

    // let actuators = Signals::new(6 * 335 + 306, n_step);
    // let m1_rbm = Signals::new(6 * 7, n_step);

    // let m2_rbm: Signals<_> = Signals::new(6 * 7, n_step);
    // let asm_cmd: Signals<_> = Signals::new(675 * 7, n_step);

    let gmt_servos =
        GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)?;

    // Scopes definition
    let mut monitor = Monitor::new();
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;

    actorscript! (
    // 1: setpoint[MountSetPoint] -> {gmt_servos::GmtMount}

    1: cfd_loads[CFDM1WindLoads] -> m1_smoother
    1: sigmoid[Weight] -> m1_smoother[CFDM1WindLoads] -> {gmt_servos::GmtFem}

    1: cfd_loads[CFDM2WindLoads] -> m2_smoother
    1: sigmoid[Weight] -> m2_smoother[CFDM2WindLoads] -> {gmt_servos::GmtFem}

    1: cfd_loads[CFDMountWindLoads] -> mount_smoother
    1: sigmoid[Weight] -> mount_smoother[CFDMountWindLoads] -> {gmt_servos::GmtFem}

    // 1: m1_rbm[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}
    // 1: actuators[assembly::M1ActuatorCommandForces] -> {gmt_servos::GmtM1}

    // 1: m2_rbm[M2RigidBodyMotions]-> {gmt_servos::GmtM2Hex}
    // 1: asm_cmd[M2ASMAsmCommand] -> {gmt_servos::GmtM2}

    8: optical_model
    1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> optical_model
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> optical_model

    40: optical_model[GuideStar].. -> stats
    40: stats[WfeRms<-9>].. -> wfe_rms_scope
    40: stats[SegmentWfeRms<-9>].. -> segment_wfe_rms_scope

    );

    Ok(())
}
