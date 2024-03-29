use std::{env, fs::File, path::Path, sync::Arc};

use crseo::{
    atmosphere,
    wavefrontsensor::{LensletArray, Pyramid},
    Atmosphere, Builder, FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    leftright::{self, Left, Right},
    once::Once,
    Average, Gain, Integrator, Offset, Signal, Signals, Timer,
};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, GuideStar, OpticalModel, Processor, PyramidCalibrator,
    PyramidMeasurements, ResidualM2modes, WavefrontSensor, WavefrontStats,
};
use gmt_dos_clients_io::optics::{
    M2modes, SegmentD7Piston, SegmentPiston, SegmentWfeRms, Wavefront, WfeRms,
};
use gmt_dos_clients_scope::server::{Monitor, Scope, Shot};
use interface::{units::NM, Data, Read, Tick, UniqueIdentifier, Update, Write, UID};
use nanorand::{Rng, WyRand};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);

    // Simulation sampling frequency (1kHz)
    let sampling_frequency = 1000_usize;

    let n_lenslet = 92;
    // let (m2_modes, n_mode) = ("ASM_DDKLs_S7OC04184_675kls", 500);
    let (m2_modes, n_mode) = ("M2_OrthoNormGS36p90_KarhunenLoeveModes", 500);

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
    let mut optical_model = OpticalModel::<Pyramid>::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, n_mode),
        )
        .source(pym.guide_stars(None))
        .sensor(pym.clone())
        .build()?;

    // Conversion from wavefront segment piston to KL piston
    let stroke = 1e-6;
    println!("Piston scaling factor: (for HDFS)");
    let data: Vec<_> = vec![vec![0f64; n_mode]; 7]
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

    let optical_model = OpticalModel::<Pyramid>::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, n_mode),
        )
        .source(pym.guide_stars(None))
        .sensor(pym.clone())
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    // Pyramid interaction matrix
    let mut pymtor = {
        let filename = format!("{0}/pym-{m2_modes}-{n_mode}.bin", env!("GMT_MODES_PATH"));
        if let Ok(pymtor) = PyramidCalibrator::try_from(filename.as_str()) {
            pymtor
        } else {
            let mut pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, n_mode)
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
    let pym_ctrl = Integrator::<Right<ResidualM2modes>>::new((n_mode - 1) * 7).gain(0.5);
    // Piston integral controller
    let piston_ctrl = Integrator::<Left<ResidualM2modes>>::new(7).gain(0.5);
    // Split piston from M2 modes
    let (split, merge) = leftright::split_merge_chunks_at::<ResidualM2modes, M2modes>(n_mode, 1);

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
    let server = |port: u32| format!("172.31.26.127:{port}");
    let mut monitor = Monitor::new();
    let segment_piston_scope = Scope::<SegmentPiston<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let segment_piston_recon_scope = Scope::<NM<SegmentPistonRecon>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let segment_piston_int_scope = Scope::<NM<SegmentPistonInt>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let n = optical_model.src.borrow().pupil_sampling();
    let pupil_scope = Shot::<Wavefront>::builder(&mut monitor, [n; 2])
        .sampling_frequency(sampling_frequency as f64 / 50f64)
        .build()?;
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let to_nm1 = Gain::new(vec![1e9; 7]);
    let to_nm3 = Gain::new(vec![1e9; 7]);

    let metronome: Timer = Timer::new(100);

    /*
       CLOSED-LOOP PYRAMID ON MODES>1
    */
    actorscript!(
        #[model(name=closed_loop_pyramid_wo_piston, )]
        #[labels(metronome = "Metronome", calibrator = "Reconstructor",
            pym_ctrl="High-orders\nintegral controller")]
        1: metronome[Tick] //-> &piston("Segment\nPiston")[SegmentPiston]
            -> optical_model[DetectorFrame]
                -> processor[PyramidMeasurements]
                    -> calibrator[ResidualM2modes]
                        -> split[Right<ResidualM2modes>]
                            -> pym_ctrl[M2modes]!
                                -> optical_model
        1: optical_model[SegmentPiston<-9>] -> segment_piston_scope
        1: optical_model[SegmentWfeRms<-9>] -> segment_wfe_rms_scope
        1: optical_model[WfeRms<-9>] -> wfe_rms_scope
        50: optical_model[Wavefront] -> pupil_scope
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
        1: metronome[Tick] //-> &piston("Segment\nPiston")[SegmentPiston]
            -> optical_model[DetectorFrame]
                -> processor[PyramidMeasurements]
                    -> calibrator[ResidualM2modes]
                        ->  split[Left<ResidualM2modes>]
                            -> to_nm1[NM<SegmentPistonRecon>] -> segment_piston_recon_scope
        1: split[Right<ResidualM2modes>]
            -> pym_ctrl[Right<ResidualM2modes>]!
                -> merge
        1: split[Left<ResidualM2modes>]
            -> piston_ctrl[Left<ResidualM2modes>]!
                -> merge
        1: merge[M2modes] -> optical_model
        1: piston_ctrl[Right<ResidualM2modes>]!
            -> to_nm3[NM<SegmentPistonInt>] -> segment_piston_int_scope
        1: optical_model[SegmentPiston<-9>] -> segment_piston_scope
        1: optical_model[SegmentWfeRms<-9>] -> segment_wfe_rms_scope
        1: optical_model[WfeRms<-9>] -> wfe_rms_scope
        50: optical_model[Wavefront] -> pupil_scope
    );

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
        1: metronome[Tick] //-> *piston("Segment\nPiston")[SegmentPiston]
            -> optical_model[DetectorFrame]
                -> processor[PyramidMeasurements]
                    -> calibrator[ResidualM2modes]
                        -> split[Left<ResidualM2modes>]
                            -> to_nm1[NM<SegmentPistonRecon>] -> segment_piston_recon_scope
        1: split[Right<ResidualM2modes>]
            -> pym_ctrl[Right<ResidualM2modes>]!
                -> merge
        1: split[Left<ResidualM2modes>]
            -> piston_ctrl[Left<ResidualM2modes>]!
                -> merge
        1: merge[M2modes] -> optical_model
        1: optical_model[SegmentPiston<-9>] -> segment_piston_scope
        1: optical_model[SegmentWfeRms<-9>] -> segment_wfe_rms_scope
        1: optical_model[WfeRms<-9>] -> wfe_rms_scope

        1: piston_ctrl[Right<ResidualM2modes>]!
            -> to_nm3[NM<SegmentPistonInt>] -> segment_piston_int_scope
        1:  optical_model[SegmentD7Piston] -> piston_sensor
        150: hdfs[SegmentAD7Piston]
            -> to_nm2[SegmentAD7Piston] -> piston_sensor_scope
        150: piston_sensor[SegmentAD7Piston]
            -> hdfs[SegmentAD7Piston] -> once
        1: once[Offset<SegmentAD7Piston>] -> piston_ctrl
        50: optical_model[Wavefront] -> pupil_scope
    );

    drop(segment_piston_scope);
    drop(segment_piston_recon_scope);
    drop(segment_piston_int_scope);
    drop(pupil_scope);
    drop(segment_wfe_rms_scope);
    drop(wfe_rms_scope);
    monitor.await?;

    Ok(())
}

#[derive(Debug, Default)]
pub struct HDFS {
    data: Arc<Vec<f64>>,
    scale: Vec<f64>,
}
impl HDFS {
    pub fn new(scale: Vec<f64>) -> Self {
        Self {
            data: Default::default(),
            scale,
        }
    }
}
impl Update for HDFS {}
impl Read<SegmentAD7Piston> for HDFS {
    fn read(&mut self, data: Data<SegmentAD7Piston>) {
        self.data = data.into_arc();
    }
}
impl Write<SegmentAD7Piston> for HDFS {
    fn write(&mut self) -> Option<Data<SegmentAD7Piston>> {
        let data: Vec<_> = self
            .data
            .iter()
            .zip(&self.scale)
            .map(|(x, s)| if x.abs() > 250e-9 { *x * s } else { 0f64 })
            .collect();
        // let mean = data.iter().sum::<f64>() / 7f64;
        // Some(
        //     data.into_iter()
        //         .map(|x| x - mean)
        //         .collect::<Vec<_>>()
        //         .into(),
        // )
        Some(data.into())
    }
}

#[derive(UID)]
#[uid(port = 60000)]
pub enum SegmentAD7Piston {}
pub enum MayBeSegmentAD7Piston {}
impl UniqueIdentifier for MayBeSegmentAD7Piston {
    type DataType = Option<Vec<f64>>;
}

#[derive(UID)]
#[uid(port = 60001)]
pub enum SegmentPistonRecon {}

#[derive(UID)]
#[uid(port = 60002)]
pub enum SegmentPistonInt {}

#[derive(UID)]
#[uid(port = 60003)]
pub enum M2modesRecon {}
