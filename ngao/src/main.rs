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
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    // Simulation sampling frequency (1kHz)
    let sampling_frequency = 1000_usize;

    let n_lenslet = 92;
    let m2_modes = "ASM_DDKLs_S7OC04184_675kls";
    let n_mode: usize = 500;

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
    let optical_model = OpticalModel::builder()
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .source(pym.guide_stars(None))
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    // Pyramid interaction matrix
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("calibrating");
    let mut pymtor: PyramidCalibrator = if !path.join("pymtor.pkl").exists() {
        let pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, n_mode)
            .n_thread(8)
            .build()?;
        serde_pickle::to_writer(
            &mut File::create(path.join("pymtor.pkl"))?,
            &pymtor,
            Default::default(),
        )?;
        pymtor
    } else {
        println!("Loading {:?}", path.join("pymtor.pkl"));
        serde_pickle::from_reader(File::open(path.join("pymtor.pkl"))?, Default::default())?
    };
    assert_eq!(n_mode, pymtor.n_mode);

    let pym = pym.build()?;
    // Pyramid data processor
    let processor: Processor<_> = Processor::from(&pym);
    // Pyramid wafefront sensor
    let pyramid = WavefrontSensor::<_, 1>::new(pym);
    // Pyramid integral controller (modes>1)
    let pym_ctrl = Integrator::<Right<ResidualM2modes>>::new((n_mode - 1) * 7).gain(0.5);
    // Piston integral controller
    let piston_ctrl = Integrator::<Left<ResidualM2modes>>::new(7).gain(0.5);
    // Split piston from M2 modes
    let (split, merge) = leftright::split_merge_chunks_at::<ResidualM2modes, M2modes>(n_mode, 1);
    // Wavefront statistics
    let stats = WavefrontStats::<1>::default();

    // Piston inputs
    // let piston: Signals = Signals::new(7, 40).channel(0, Signal::Constant(25e-9));
    let mut rng = WyRand::new(); //_seed(4237);
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
    });

    // Scopes definition
    let server = |port: u32| format!("172.31.26.127:{port}");
    let mut monitor = Monitor::new();
    let segment_piston_scope = Scope::<SegmentPiston<-9>>::builder(server(5001), &mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let segment_piston_recon_scope =
        Scope::<NM<SegmentPistonRecon>>::builder(server(5002), &mut monitor)
            .sampling_frequency(sampling_frequency as f64)
            .build()?;
    let segment_piston_int_scope =
        Scope::<NM<SegmentPistonInt>>::builder(server(5004), &mut monitor)
            .sampling_frequency(sampling_frequency as f64)
            .build()?;
    let n = optical_model.src.lock().unwrap().pupil_sampling();
    let pupil_scope = Shot::<Wavefront>::builder(server(5005), &mut monitor, [n; 2])
        .sampling_frequency(sampling_frequency as f64 / 50f64)
        .build()?;
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(server(5006), &mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(server(5007), &mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let to_nm1 = Gain::new(vec![1e9; 7]);
    let to_nm3 = Gain::new(vec![1e9; 7]);

    let metronome: Timer = Timer::new(100);
    // Pyramid reconstructor
    pymtor.h00_estimator()?;
    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    actorscript!(
        #[model(name=closed_loop_pyramid_wo_piston, state = running)]
        1: metronome("Metronome")[Tick] //-> &piston("Segment\nPiston")[SegmentPiston]
            -> &optical_model[GuideStar] -> &stats[SegmentPiston<-9>] -> &segment_piston_scope
        1: &stats[SegmentWfeRms<-9>] -> &segment_wfe_rms_scope
        1: &stats[WfeRms<-9>] -> &wfe_rms_scope
        1: &optical_model[GuideStar]
            -> &pyramid[DetectorFrame<f32>]
                -> &processor[PyramidMeasurements] -> calibrator("Reconstructor")
        1: calibrator("Reconstructor")[Right<ResidualM2modes>]
            -> &pym_ctrl("High-orders\nintegral controller")[M2modes]!
                 -> &optical_model
        50: &optical_model[Wavefront] -> &pupil_scope
    );

    let metronome: Timer = Timer::new(100);
    // Pyramid reconstructor
    let pym_constrained_recon: nalgebra::DMatrix<f32> = serde_pickle::from_reader(
        File::open(path.join("pym_constrained_recon.pkl"))?,
        Default::default(),
    )?;
    pymtor.set_hp_estimator(pym_constrained_recon);
    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    closed_loop_pyramid_wo_piston.await?;

    actorscript!(
        #[model(name=closed_loop_pyramid, state = running, resume = True)]
        1: metronome("Metronome")[Tick] //-> &piston("Segment\nPiston")[SegmentPiston]
            -> *optical_model[GuideStar] -> *stats[SegmentPiston<-9>] -> *segment_piston_scope
        1: *stats[SegmentWfeRms<-9>] -> *segment_wfe_rms_scope
        1: *stats[WfeRms<-9>] -> *wfe_rms_scope
        1: *optical_model[GuideStar]
            -> *pyramid[DetectorFrame<f32>]
                -> *processor[PyramidMeasurements] -> &calibrator("Reconstructor")
        1: &calibrator("Reconstructor")[ResidualM2modes]
            ->  &split("Extract piston\nfrom M2 modes")[Left<ResidualM2modes>]
                -> &to_nm1("nm")[NM<SegmentPistonRecon>] -> &segment_piston_recon_scope
        1: &split("Extract piston\nfrom M2 modes")[Right<ResidualM2modes>]
            -> *pym_ctrl("High-orders\nintegral controller")[Right<ResidualM2modes>]!
                -> &merge("Merge piston\nwith other modes")
        1: &split("Extract piston\nfrom M2 modes")[Left<ResidualM2modes>]
            -> &piston_ctrl("Piston integral\ncontroller")[Left<ResidualM2modes>]!
                -> &merge("Merge piston\nwith other modes")
        1: &merge("Merge piston\nwith other modes")[M2modes] -> *optical_model
        1: &piston_ctrl("Piston integral\ncontroller")[Right<ResidualM2modes>]!
            -> &to_nm3("nm")[NM<SegmentPistonInt>] -> &segment_piston_int_scope
        50: *optical_model[Wavefront] -> *pupil_scope
    );

    // Diferential piston averager
    let piston_sensor = Average::<f64, SegmentD7Piston, SegmentAD7Piston>::new(7);
    let to_nm2 = Gain::new(vec![1e9; 7]);
    let piston_sensor_scope =
        Scope::<SegmentAD7Piston>::builder("172.31.26.127:5003", &mut monitor)
            .sampling_frequency((sampling_frequency / 150) as f64)
            .build()?;
    let metronome: Timer = Timer::new(800 + 2000);
    let once = Once::new();
    let hdfs = HDFS::default();

    closed_loop_pyramid.await?;

    actorscript!(
        #[model(name = closed_loop_pyramid_hdfs, state = completed, resume = True)]
        1: metronome("Metronome")[Tick] //-> *piston("Segment\nPiston")[SegmentPiston]
            -> *optical_model[GuideStar] -> *stats[SegmentPiston<-9>] -> *segment_piston_scope
        1: *stats[SegmentWfeRms<-9>] -> *segment_wfe_rms_scope
        1: *stats[WfeRms<-9>] -> *wfe_rms_scope
        1: *optical_model[GuideStar]
            -> *pyramid[DetectorFrame<f32>]
                -> *processor[PyramidMeasurements] -> *calibrator("Reconstructor")
        1: *calibrator("Reconstructor")[ResidualM2modes]
            ->  *split("Extract piston\nfrom M2 modes")[Left<ResidualM2modes>]
                -> *to_nm1("nm")[NM<SegmentPistonRecon>] -> *segment_piston_recon_scope
        1: *split("Extract piston\nfrom M2 modes")[Right<ResidualM2modes>]
            -> *pym_ctrl("High-orders\nintegral controller")[Right<ResidualM2modes>]!
                -> *merge("Merge piston\nwith other modes")
        1: *split("Extract piston\nfrom M2 modes")[Left<ResidualM2modes>]
            -> *piston_ctrl("Piston integral\ncontroller")[Left<ResidualM2modes>]!
                -> *merge("Merge piston\nwith other modes")
        1: *merge("Merge piston\nwith other modes")[M2modes] -> *optical_model
        1: *piston_ctrl("Piston integral\ncontroller")[Right<ResidualM2modes>]!
            -> *to_nm3("nm")[NM<SegmentPistonInt>] -> *segment_piston_int_scope
        1:  *stats[SegmentD7Piston] -> piston_sensor("HDFS differential\npiston average")
        150: hdfs("HDFS piston\n> half a wave")[SegmentAD7Piston]
            -> to_nm2("nm")[SegmentAD7Piston] -> piston_sensor_scope
        150: piston_sensor("HDFS differential\npiston average")[SegmentAD7Piston]
            -> hdfs("HDFS piston\n> half a wave")[SegmentAD7Piston] -> once
        1: once[Offset<SegmentAD7Piston>] -> *piston_ctrl("Piston integral\ncontroller")
        50: *optical_model[Wavefront] -> *pupil_scope
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
            .map(|x| if x.abs() > 250e-9 { *x } else { 0f64 })
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
pub enum SegmentAD7Piston {}
pub enum MayBeSegmentAD7Piston {}
impl UniqueIdentifier for MayBeSegmentAD7Piston {
    type DataType = Option<Vec<f64>>;
}

#[derive(UID)]
pub enum SegmentPistonRecon {}

#[derive(UID)]
pub enum SegmentPistonInt {}

#[derive(UID)]
pub enum M2modesRecon {}
