use std::{env, path::Path};

use crseo::{gmt::GmtM2, imaging::LensletArray, Atmosphere, FromBuilder, Gmt, RayTracing};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif, print::Print, Integrator, Timer};
use gmt_dos_clients_crseo::{
    calibration::{Calibration, CalibrationMode},
    centroiding::CentroidsProcessing,
    sensors::Camera,
    DeviceInitialize, OpticalModel,
};
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_m2::asm::{
        M2ASMAsmCommand, M2ASMFaceSheetFigure, M2ASMFluidDampingForces, M2ASMVoiceCoilsForces,
        M2ASMVoiceCoilsMotion,
    },
    optics::{Dev, Frame, M2modes, SegmentWfeRms, SensorData, Wavefront, WfeRms},
};
use gmt_dos_clients_m2_ctrl::ASMS;
use interface::Tick;
use ltao::{
    m2_parameters::{ASM_N_ACTUATOR, M2_N_MODE},
    ModalToZonal,
};

const N_STEP: usize = 101;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("asms_closed-loop_atmosphere");
    env::set_var("DATA_REPO", &data_path);

    let sampling_frequency = 8000;
    // let sim_duration = 1_usize; // second
    // let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = gmt_fem::FEM::from_env().unwrap();

    let asms = ASMS::<1>::new(&mut fem)?.build()?;
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_asms(Some(vec![1, 2, 3, 4, 5, 6, 7]), None, None)?
        .outs_by_name((1..=7).map(|i| format!("M2_segment_{i}_axial_d")).collect())?
        .build()?;
    println!("{plant}");
    // c            self.outs_by_name((1..=7).map(|i| format!("M2_segment_{i}_axial_d")).collect())?

    // let gmt = Gmt::builder().m2("Karhunen-Loeve", M2_N_MODE);
    let gmt = Gmt::builder().m2("asms_eigenmodes_wz123", M2_N_MODE);

    let atm_builder = Atmosphere::builder()
        .single_turbulence_layer(0f32, Some(7f32), Some(0f32))
        .ray_tracing(
            RayTracing::default().duration(1.), // .filepath("atm_single_layer.bin"),
        );

    let sensor =
        Camera::builder().lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(8));

    let mut centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sensor)?;

    let om = OpticalModel::<Camera>::builder()
        .gmt(gmt.clone())
        .sensor(sensor);

    let mut recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(&om).into(),
        CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(2),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    om.initialize(&mut centroids);

    let om = om
        .gmt(gmt.clone().m2("asms_ifs_gmt-fem", ASM_N_ACTUATOR))
        .sampling_frequency(sampling_frequency as f64)
        .atmosphere(atm_builder)
        .build()?;

    let int = Integrator::new(M2_N_MODE * 7).gain(0.5);

    // Modal to zonal conversion
    let modes2actuators = ModalToZonal::new().unwrap();

    let wavefront = gif::Frame::<f64>::new("wavefront.png", om.source().pupil_sampling());

    let timer: Timer = Timer::new(N_STEP);
    let print = Print::default();
    actorscript!(
        #[model(name=closed_loop_atmosphere)]
        1: timer[Tick]
            -> om[Frame<Dev>]!
                -> centroids[SensorData]
                    -> recon[M2modes]
                        -> int[M2modes]
                            -> modes2actuators
        1: modes2actuators[M2ASMAsmCommand] -> {asms}[M2ASMVoiceCoilsForces]-> plant
        1: {asms}[M2ASMFluidDampingForces] -> plant[M2ASMVoiceCoilsMotion]! -> {asms}
        1: plant[M2ASMFaceSheetFigure]! -> om

        1: om[WfeRms<-9>] -> print
        1: om[SegmentWfeRms<-9>] -> print
        // 1: om[SegmentPiston<-9>]
        100: om[Wavefront] -> wavefront
    );

    wavefront.lock().await.save()?;

    Ok(())
}
