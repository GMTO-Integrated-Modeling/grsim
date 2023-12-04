use std::{env, fs::File, mem, path::Path, sync::Arc};

use crseo::{
    wavefrontsensor::{LensletArray, Pyramid},
    Builder, FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    once::Once, select::Select, Average, Gain, Integrator, Signal, Signals, Timer,
};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, GuideStar, OpticalModel, Processor, PyramidCalibrator,
    PyramidMeasurements, ResidualM2modes, WavefrontSensor, WavefrontStats,
};
use gmt_dos_clients_io::optics::{M2modes, SegmentD7Piston, SegmentPiston};
use gmt_dos_clients_scope::server::{Monitor, Scope};
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

    let sampling_frequency = 1000_usize;

    let n_lenslet = 92;
    let m2_modes = "ASM_DDKLs_S7OC04184_675kls";
    let n_mode: usize = 500; //env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);

    let optical_model = OpticalModel::builder()
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .source(pym.guide_stars(None))
        // .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

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

    let pym_constrained_recon: nalgebra::DMatrix<f32> = serde_pickle::from_reader(
        File::open(path.join("pym_constrained_recon.pkl"))?,
        Default::default(),
    )?;
    pymtor.set_hp_estimator(pym_constrained_recon);
    // pymtor.hp_estimator()?;
    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    let pym = pym.build()?;
    let processor: Processor<_> = Processor::from(&pym);
    let pyramid = WavefrontSensor::<_, 1>::new(pym);
    let pym_ctrl = Integrator::<ResidualM2modes>::new(n_mode * 7).gain(0.5);

    let stats = WavefrontStats::<1>::default();

    // let piston: Signals = Signals::new(7, 40).channel(6, Signal::Constant(50e-9));
    let mut rng = WyRand::new(); //_seed(4237);
                                 // let mut random_piston = |a: f64| dbg!((2. * rng.generate::<f64>() - 1.) * a);
    let piston: Signals = (0..7).fold(Signals::new(7, usize::MAX), |signals, i| {
        signals.channel(
            i,
            Signal::Constant((2. * rng.generate::<f64>() - 1.) * 2.5e-6), /* + Signal::Sinusoid {
                                                                              amplitude: 25e-9,
                                                                              sampling_frequency_hz: sampling_frequency as f64,
                                                                              frequency_hz: 10.,
                                                                              phase_s: rng.generate::<f64>(),
                                                                          }, */
        )
    });

    let mut monitor = Monitor::new();
    let segment_piston_scope =
        Scope::<SegmentPiston<-9>>::builder("172.31.26.127:5001", &mut monitor)
            .sampling_frequency(sampling_frequency as f64)
            .build()?;
    let segment_piston_recon_scope =
        Scope::<NM<SegmentPistonRecon>>::builder("172.31.26.127:5002", &mut monitor)
            .sampling_frequency(sampling_frequency as f64)
            .build()?;
    let segment_piston_int_scope =
        Scope::<NM<SegmentPistonInt>>::builder("172.31.26.127:5004", &mut monitor)
            .sampling_frequency(sampling_frequency as f64)
            .build()?;

    let select_recon = Select::<f64>::new((0..7).map(|i| i * n_mode).collect::<Vec<usize>>());
    let select_int = Select::<f64>::new((0..7).map(|i| i * n_mode).collect::<Vec<usize>>());
    let to_nm1 = Gain::new(vec![1e9; 7]);
    let to_nm3 = Gain::new(vec![1e9; 7]);

    let metronome: Timer = Timer::new(100);

    actorscript!(
        #[model(state = completed)]
        1: metronome[Tick] -> &piston("Segment\nPiston")[SegmentPiston]
            -> &optical_model[GuideStar] -> &stats[SegmentPiston<-9>] -> &segment_piston_scope
        1: &optical_model[GuideStar]
            -> &pyramid[DetectorFrame<f32>]
                -> &processor[PyramidMeasurements]
                    -> &calibrator("Reconstructor")[ResidualM2modes]
                        -> &pym_ctrl[M2modes]! -> &optical_model
        1: &calibrator("Reconstructor")[ResidualM2modes]
            -> &select_recon("Reconstructed\nSegment\nPiston")[SegmentPistonRecon]
                -> &to_nm1("nm")[NM<SegmentPistonRecon>] -> &segment_piston_recon_scope
        1: &pym_ctrl[M2modes]! -> &select_int("Reconstructed\nIntegrated\nPiston")[SegmentPistonInt]
         -> &to_nm3("nm")[NM<SegmentPistonInt>] -> &segment_piston_int_scope
    );

    let piston_sensor = Average::<f64, SegmentD7Piston, SegmentAD7Piston>::new(7);
    let piston_sensor_scope =
        Scope::<SegmentAD7Piston>::builder("172.31.26.127:5003", &mut monitor)
            .sampling_frequency((sampling_frequency / 25) as f64)
            .build()?;
    let metronome: Timer = Timer::new(200);
    let to_nm2 = Gain::new(vec![1e9; 7]);
    let once = Once::new();
    let merger = Merger::new(n_mode);

    actorscript!(
    #[model(state = completed, resume = True)]
        1: metronome[Tick] -> *piston("Segment\nPiston")[SegmentPiston]
            -> *optical_model[GuideStar] -> *stats[SegmentPiston<-9>] -> *segment_piston_scope
        1: *optical_model[GuideStar]
            -> *pyramid[DetectorFrame<f32>]
                -> *processor[PyramidMeasurements]
                    -> *calibrator("Reconstructor")[ResidualM2modes]
                        -> merger
        1: *calibrator("Reconstructor")[ResidualM2modes]
            -> *select_recon("Reconstructed\nSegment\nPiston")[SegmentPistonRecon]
                -> *to_nm1("nm")[NM<SegmentPistonRecon>] -> *segment_piston_recon_scope
        1:  *stats[SegmentD7Piston] -> piston_sensor
        25: piston_sensor[SegmentAD7Piston] -> to_nm2("nm")[SegmentAD7Piston] -> piston_sensor_scope
        25: piston_sensor[SegmentAD7Piston] -> once
        1: once[MayBeSegmentAD7Piston] -> merger[ResidualM2modes] -> *pym_ctrl[M2modes]! -> *optical_model
        1: *pym_ctrl[M2modes]! -> *select_int("Reconstructed\nIntegrated\nPiston")[SegmentPistonInt]
         -> *to_nm3("nm")[NM<SegmentPistonInt>] -> *segment_piston_int_scope
    );

    drop(segment_piston_scope);
    drop(segment_piston_recon_scope);
    drop(segment_piston_int_scope);
    monitor.await?;

    Ok(())
}

#[derive(Debug, Default)]
pub struct Merger {
    n_mode: usize,
    hdfs: Arc<Option<Vec<f64>>>,
    pym: Vec<f64>,
}

impl Merger {
    pub fn new(n_mode: usize) -> Self {
        Self {
            n_mode,
            ..Default::default()
        }
    }
}

impl Update for Merger {}

impl Read<ResidualM2modes> for Merger {
    fn read(&mut self, data: Data<ResidualM2modes>) {
        self.pym = data.into();
    }
}

impl Write<ResidualM2modes> for Merger {
    fn write(&mut self) -> Option<Data<ResidualM2modes>> {
        if let Some(hdfs) = self.hdfs.as_ref() {
            self.pym
                .chunks_mut(self.n_mode)
                .zip(hdfs.iter())
                .for_each(|(m, dp)| {
                    if (*dp).abs() > 250e-9 {
                        m[0] += dp;
                    }
                });
            // let piston = self.pym.chunks(self.n_mode).map(|x| x[0]).sum::<f64>() / 7f64;
            // self.pym
            //     .chunks_mut(self.n_mode)
            //     .for_each(|x| x[0] -= piston);
        }
        Some(mem::take(&mut self.pym).into())
    }
}

impl Read<MayBeSegmentAD7Piston> for Merger {
    fn read(&mut self, data: Data<MayBeSegmentAD7Piston>) {
        self.hdfs = data.into_arc();
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
