use std::{env, path::Path};

use crseo::{
    atmosphere,
    wavefrontsensor::{LensletArray, Pyramid},
    Atmosphere, FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    leftright::{self, Left, Right},
    once::Once,
    Average, Gain, Integrator, Offset, Signal, Signals, Timer,
};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, OpticalModel, Processor, PyramidCalibrator, PyramidMeasurements,
    ResidualM2modes,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::{M2ASMAsmCommand, M2ASMFaceSheetFigure, M2ASMReferenceBodyNodes},
    optics::{M2modes, SegmentD7Piston, SegmentPiston, SegmentWfeRms, Wavefront, WfeRms},
};
use gmt_dos_clients_scope::server::{Monitor, Scope, Shot};
use gmt_dos_clients_servos::{asms_servo::ReferenceBody, AsmsServo, GmtM2, GmtServoMechanisms};
use gmt_dos_clients_transceiver::Transceiver;
use gmt_fem::FEM;
use interface::{filing::Filing, units::NM, Data, Read, Tick, Update, Write};

use ngao_opm::{
    hdfs::HDFS, modal_to_zonal::ModalToZonal, SegmentAD7Piston, SegmentPistonInt,
    SegmentPistonRecon, ACTUATOR_RATE, N_ACTUATOR, N_MODE,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    env::set_var("DATA_REPO", &data_repo);

    // Simulation sampling frequency (1kHz)
    let sampling_frequency = 8000;
    let optical_model_sampling_frequency = 1000;

    let n_lenslet = 92;
    // let (m2_modes, n_mode) = ("ASM_DDKLs_S7OC04184_675kls", 500);
    let m2_modes = "M2_OrthoNormGS36p90_KarhunenLoeveModes";

    // Pyramid definition
    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);
    // Optical Model (GMT with 1 guide star on-axis)
    // Atmospher builder
    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(5f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );

    // Conversion from wavefront segment piston to KL piston
    let mut optical_model = OpticalModel::<Pyramid>::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, N_MODE),
        )
        .source(pym.guide_stars(None))
        .sensor(pym.clone())
        .build()?;

    let stroke = 1e-6;
    println!("Piston scaling factor: (for HDFS)");
    let data: Vec<_> = vec![vec![0f64; N_MODE]; 7]
        .into_iter()
        .flat_map(|mut modes| {
            modes[0] = stroke;
            modes
        })
        .collect();
    <OpticalModel<Pyramid> as Read<M2modes>>::read(&mut optical_model, Data::new(data));
    optical_model.update();
    let piston_scale: Vec<_> =
        <OpticalModel<Pyramid> as Write<SegmentPiston>>::write(&mut optical_model)
            .map(|sp| {
                (*sp)
                    .iter()
                    .map(|sp| (sp / stroke).recip())
                    .inspect(|x| println!("{}", x))
                    .collect()
            })
            .unwrap();

    // Pyramid interaction matrix
    let mut pymtor = {
        let filename = format!("{0}/pym-{m2_modes}-{N_MODE}.bin", env!("GMT_MODES_PATH"));
        if let Ok(pymtor) = PyramidCalibrator::try_from(filename.as_str()) {
            pymtor
        } else {
            let mut pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, N_MODE)
                .n_gpu(7)
                .stroke(25e-7)
                .build()?;
            pymtor.h00_estimator()?;
            pymtor.save(filename)?
        }
    };

    // Pyramid data processor
    let processor: Processor<_> = Processor::try_from(&pym)?;
    // Pyramid integral controller (modes>1)
    let pym_ctrl = Integrator::<Right<ResidualM2modes>>::new((N_MODE - 1) * 7).gain(0.5);
    // Piston integral controller
    let piston_ctrl = Integrator::<Left<ResidualM2modes>>::new(7).gain(0.5);
    // Split piston from M2 modes
    let (split, merge) = leftright::split_merge_chunks_at::<ResidualM2modes, M2modes>(N_MODE, 1);

    // Piston inputs
    let piston: Signals = Signals::new(7, 40).channel(0, Signal::Constant(25e-9));
    /*     let mut rng = WyRand::new(); //_seed(4237);
                                 // let mut random_piston = |a: f64| dbg!((2. * rng.generate::<f64>() - 1.) * a);
    let piston: Signals = (0..7).fold(Signals::new(7, usize::MAX), |signals, i| {
        signals.channel(
            i,
            Signal::Constant((2. * rng.generate::<f64>() - 1.) * 2.5e-6)
                + Signal::Sinusoid {
                    amplitude: 350e-9,
                    sampling_frequency_hz: sampling_frequency as f64,
                    frequency_hz: 10.,
                    phase_s: rng.generate::<f64>(),
                },
        )
    }); */

    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    // Scopes definition
    let mut monitor = Monitor::new();
    let segment_piston_scope = Scope::<SegmentPiston<-9>>::builder(&mut monitor)
        .sampling_frequency(optical_model_sampling_frequency as f64)
        .build()?;
    let segment_piston_recon_scope = Scope::<NM<SegmentPistonRecon>>::builder(&mut monitor)
        .sampling_frequency(optical_model_sampling_frequency as f64)
        .build()?;
    let segment_piston_int_scope = Scope::<NM<SegmentPistonInt>>::builder(&mut monitor)
        .sampling_frequency(optical_model_sampling_frequency as f64)
        .build()?;
    let n = optical_model.src.borrow().pupil_sampling();
    let pupil_scope = Shot::<Wavefront>::builder(&mut monitor, [n; 2])
        .sampling_frequency(optical_model_sampling_frequency as f64 / 50f64)
        .build()?;
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(optical_model_sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(optical_model_sampling_frequency as f64)
        .build()?;

    let to_nm1 = Gain::new(vec![1e9; 7]);
    let to_nm3 = Gain::new(vec![1e9; 7]);

    let optical_model = OpticalModel::<Pyramid>::builder()
        .gmt(
            Gmt::builder()
                // .m1_truss_projection(false)
                .m2("ASM_IFs_permutated_90", N_ACTUATOR),
        )
        .source(pym.guide_stars(None))
        .sensor(pym.clone())
        .atmosphere(atm_builder)
        .sampling_frequency(optical_model_sampling_frequency as f64)
        .build()?;

    // GMT Sevomechanisms actors
    /*     let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path_or_else(
        Path::new(env!("FEM_REPO")).join("servos.bin"),
        || {
            GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(
                sampling_frequency as f64,
                FEM::from_env().unwrap(),
            )
            //.wind_loads(WindLoads::new())
            .asms_servo(
                AsmsServo::new()
                    .facesheet(Default::default())
                    .reference_body(ReferenceBody::new()),
            )
        },
    )?; */

    // Modal to zonal conversion
    let modes2actuators = ModalToZonal::new().unwrap();
    let metronome: Timer = Timer::new(100);

    let null_piston = Signals::new(7, 100);

    // TRANSCEIVER ...
    let server_address = env::var("RX_SERVER_ADDRESS")?;
    let client_address = env::var("TX_ADDRESS")?;
    let mut monitor = Monitor::new();

    let m2_asm_cmd =
        Transceiver::<M2ASMAsmCommand>::transmitter(&client_address)?.run(&mut monitor);

    let rx_address = "0.0.0.0:0";
    let m2_asm_facesheet =
        Transceiver::<M2ASMFaceSheetFigure>::receiver(&server_address, rx_address)?
            .run(&mut monitor);
    let m1_rbms =
        Transceiver::<M1RigidBodyMotions>::receiver(&server_address, rx_address)?.run(&mut monitor);
    let m2_rbms = Transceiver::<M2ASMReferenceBodyNodes>::receiver(&server_address, rx_address)?
        .run(&mut monitor);
    // ... TRANSCEIVER

    /*
       CLOSED-LOOP PYRAMID ON MODES>1
    */
    actorscript!(
        #[model(name=closed_loop_pyramid_wo_piston, )]
        #[labels(metronome = "Metronome", calibrator = "Reconstructor",
            pym_ctrl="High-orders\nintegral controller")]
        8: metronome[Tick] //-> &piston("Segment\nPiston")[SegmentPiston]
            -> optical_model[DetectorFrame]
                -> processor[PyramidMeasurements]
                    -> calibrator[ResidualM2modes]
                        -> split[Right<ResidualM2modes>]
                            -> pym_ctrl[Right<ResidualM2modes>]!
                                -> merge
        8: null_piston[Left<ResidualM2modes>] -> merge
        8: merge[M2modes]  -> modes2actuators

        // 1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: modes2actuators[M2ASMAsmCommand] -> m2_asm_cmd
        // 1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> optical_model
        1: m2_asm_facesheet[M2ASMFaceSheetFigure] -> optical_model

        // 1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> optical_model
        1: m1_rbms[M1RigidBodyMotions] -> optical_model
        // 1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes] -> optical_model
        1: m2_rbms[M2ASMReferenceBodyNodes] -> optical_model

        8: optical_model[SegmentPiston<-9>] -> segment_piston_scope
        8: optical_model[SegmentWfeRms<-9>] -> segment_wfe_rms_scope
        8: optical_model[WfeRms<-9>] -> wfe_rms_scope
        400: optical_model[Wavefront] -> pupil_scope
    );

    let metronome: Timer = Timer::new(100);
    // Pyramid reconstructor
    // let pym_constrained_recon: nalgebra::DMatrix<f32> = serde_pickle::from_reader(
    //     File::open(data_repo.join(format!("pym_{m2_modes}_{n_mode}_constrained_recon.pkl")))?,
    //     Default::default(),
    // )?;
    // pymtor.set_hp_estimator(pym_constrained_recon);
    pymtor.hp_estimator()?;
    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    // closed_loop_pyramid_wo_piston.await?;

    /*
       CLOSED-LOOP PYRAMID WITH 2 INTEGRAL CONTROLLERS
       One controller for modes>1, the other for piston
    */
    actorscript!(
        #[model(name=closed_loop_pyramid)]
        #[labels(metronome = "Metronome", calibrator = "Reconstructor",
            pym_ctrl="High-orders\nintegral controller",
            split = "Extract piston\nfrom M2 modes",
            to_nm1 = "nm", to_nm3 = "nm",
            merge = "Merge piston\nwith other modes",
            piston_ctrl = "Piston integral\ncontroller")]
        8: metronome[Tick] //-> &piston("Segment\nPiston")[SegmentPiston]
            -> optical_model[DetectorFrame]
                -> processor[PyramidMeasurements]
                    -> calibrator[ResidualM2modes]
                        ->  split[Left<ResidualM2modes>]
                            -> to_nm1[NM<SegmentPistonRecon>] -> segment_piston_recon_scope
        8: split[Right<ResidualM2modes>]
            -> pym_ctrl[Right<ResidualM2modes>]!
                -> merge
        8: split[Left<ResidualM2modes>]
            -> piston_ctrl[Left<ResidualM2modes>]!
                -> merge
        8: merge[M2modes] -> modes2actuators

        // 1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: modes2actuators[M2ASMAsmCommand] -> m2_asm_cmd
        // 1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> optical_model
        1: m2_asm_facesheet[M2ASMFaceSheetFigure] -> optical_model

        // 1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> optical_model
        1: m1_rbms[M1RigidBodyMotions] -> optical_model
        // 1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes] -> optical_model
        1: m2_rbms[M2ASMReferenceBodyNodes] -> optical_model

        8: piston_ctrl[Right<ResidualM2modes>]!
            -> to_nm3[NM<SegmentPistonInt>] -> segment_piston_int_scope
        8: optical_model[SegmentPiston<-9>] -> segment_piston_scope
        8: optical_model[SegmentWfeRms<-9>] -> segment_wfe_rms_scope
        8: optical_model[WfeRms<-9>] -> wfe_rms_scope
        400: optical_model[Wavefront] -> pupil_scope
    );
    /*
       // Diferential piston averager
       let piston_sensor = Average::<f64, SegmentD7Piston, SegmentAD7Piston>::new(7);
       let to_nm2 = Gain::new(vec![1e9; 7]);
       let piston_sensor_scope = Scope::<SegmentAD7Piston>::builder(&mut monitor)
           .sampling_frequency((sampling_frequency / 150) as f64)
           .build()?;
       let metronome: Timer = Timer::new(800 + 4000);
       let once = Once::new();
       let hdfs = HDFS::new(piston_scale);

       // closed_loop_pyramid.await?;

       /*
          CLOSED-LOOP PYRAMID WITH 2 INTEGRAL CONTROLLERS & HDFS WAVE CATCHER
          One controller for modes>1, the other for piston
       */
       actorscript!(
           #[model(name = closed_loop_pyramid_hdfs)]
           #[labels(metronome = "Metronome", calibrator = "Reconstructor",
               pym_ctrl="High-orders\nintegral controller",
               split = "Extract piston\nfrom M2 modes",
               to_nm1 = "nm", to_nm2 = "nm", to_nm3 = "nm",
               merge = "Merge piston\nwith other modes",
               piston_ctrl = "Piston integral\ncontroller",
               piston_sensor = "HDFS differential\npiston average",
               hdfs = "HDFS piston\n> half a wave")]
           8: metronome[Tick] //-> *piston("Segment\nPiston")[SegmentPiston]
               -> optical_model[DetectorFrame]
                   -> processor[PyramidMeasurements]
                       -> calibrator[ResidualM2modes]
                           -> split[Left<ResidualM2modes>]
                               -> to_nm1[NM<SegmentPistonRecon>] -> segment_piston_recon_scope
           8: split[Right<ResidualM2modes>]
               -> pym_ctrl[Right<ResidualM2modes>]!
                   -> merge
           8: split[Left<ResidualM2modes>]
               -> piston_ctrl[Left<ResidualM2modes>]!
                   -> merge
           8: merge[M2modes] -> modes2actuators

           1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
           1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> optical_model

           1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> optical_model
           1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes] -> optical_model

           8: optical_model[SegmentPiston<-9>] -> segment_piston_scope
           8: optical_model[SegmentWfeRms<-9>] -> segment_wfe_rms_scope
           8: optical_model[WfeRms<-9>] -> wfe_rms_scope

           8: piston_ctrl[Right<ResidualM2modes>]!
               -> to_nm3[NM<SegmentPistonInt>] -> segment_piston_int_scope
           8:  optical_model[SegmentD7Piston] -> piston_sensor
           1200: hdfs[SegmentAD7Piston]
               -> to_nm2[SegmentAD7Piston] -> piston_sensor_scope
           1200: piston_sensor[SegmentAD7Piston]
               -> hdfs[SegmentAD7Piston] -> once
           8: once[Offset<SegmentAD7Piston>] -> piston_ctrl
           400: optical_model[Wavefront] -> pupil_scope
       );
    */
    drop(segment_piston_scope);
    drop(segment_piston_recon_scope);
    drop(segment_piston_int_scope);
    drop(pupil_scope);
    drop(segment_wfe_rms_scope);
    drop(wfe_rms_scope);
    monitor.await?;

    Ok(())
}
