/*!
# M2 RBMS

In this example, we move the ASMS S1 reference body by 1 micron along the local z-axis (Tz)
and record the rigid body motions and axial displacements of the ASMS S1 facesheet

Run the example with:

```bash
cargo run --release --example m2-rbms --features s8000d002ze30 --no-default-features
```

and post-process with:

```python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_parquet("model-data_40.parquet")
plt.figure();plt.plot(np.vstack(df["M2RigidBodyMotions"]));

df = pd.read_parquet("model-data_3999.parquet")
plt.figure();plt.plot(np.vstack(df["FaceSheetFigure#1"])[-1,:],'.');
```
*/

use std::{env, path::Path};

use crseo::{FromBuilder, Gmt};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{Signal, Signals, Tick, Timer};
use gmt_dos_clients_crseo::{sensors::NoSensor, OpticalModel};
use gmt_dos_clients_io::{
    gmt_m1::{
        assembly::{M1ActuatorCommandForces, M1ModeCoefficients},
        M1ModeShapes, M1RigidBodyMotions,
    },
    optics::Wavefront,
};
use gmt_dos_clients_servos::{GmtFem, GmtM1, GmtServoMechanisms, M1SegmentFigure};
//asms_servo
use gmt_fem::FEM;
use interface::filing::Filing;
use ltao::{m1_parameters::M1_N_MODE, M1BendingModes, ModalToZonal};
// use nanorand::{Rng, WyRand};

const ACTUATOR_RATE: usize = 80; //100Hz

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples")
            .join("m1-bms"),
    );

    // let mut rng = WyRand::new();

    let sim_sampling_frequency = 8000;
    let n_step = sim_sampling_frequency / 2;

    let fem = FEM::from_env()?;

    // M2 S1
    // let m1_rbm: Signals = (0..42).fold(Signals::new(42, n_step), |signals, i| {
    //     signals.channel(
    //         i,
    //         Signal::Sigmoid {
    //             amplitude: 1e-6 * (rng.generate::<f64>() * 2. - 1.),
    //             sampling_frequency_hz: sim_sampling_frequency as f64,
    //         },
    //     )
    // });
    let sigmoid = Signal::Sigmoid {
        amplitude: 1e-5,
        sampling_frequency_hz: sim_sampling_frequency as f64,
    };
    let m1_bm = Signals::new(7 * M1_N_MODE, n_step).channel(0, 1e-7);

    // GMT Servo-mechanisms system

    let gmt_servos =
        Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo_or_else("servos.bin", || {
            GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
                // .asms_servo(AsmsServo::new().facesheet(Default::default()))
                .m1_segment_figure(M1SegmentFigure::new())
        })?;

    // let gmt_servos =
    //     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
    //         .asms_servo(
    //             AsmsServo::new()
    //                 .facesheet(asms_servo::Facesheet::new().options(Box::new(MyFacesheet))),
    //         )
    //         .build()?;

    let modes_to_forces = ModalToZonal::m1().unwrap();
    let m1_bms = M1BendingModes::new()?;

    let om = OpticalModel::<NoSensor>::builder()
        .gmt(Gmt::builder().m1("20240401_1605_m1_raw_bending_modes", 27))
        .build()?;

    actorscript! {
        1: m1_bm[M1ModeShapes] -> modes_to_forces[M1ActuatorCommandForces] -> {gmt_servos::GmtM1}
    };

    // gmt_servos.to_path("servos.bin")?;
    // let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path("servos.bin")?;

    let nope: Timer = Timer::new(0);
    actorscript! {
        #[model(name=m1_bms)]
        1: nope[Tick] -> {gmt_servos::GmtFem}[M1RigidBodyMotions]!$
        1: nope[Tick] -> {gmt_servos::GmtFem}[M1ModeShapes]!${6*602+579}
            -> m1_bms[M1ModeCoefficients]${7*335}
                -> om[Wavefront]$
        // 1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<2>]!${602}
        // 1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<3>]!${602}
        // 1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<4>]!${602}
        // 1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<5>]!${602}
        // 1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<6>]!${602}
        // 1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<7>]!${579}
    }

    Ok(())
}
