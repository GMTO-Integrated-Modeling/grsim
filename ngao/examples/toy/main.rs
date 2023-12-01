use std::{env, fs::File, marker::PhantomData, ops::Range, path::Path, sync::Arc};

use crseo::{
    wavefrontsensor::{LensletArray, Pyramid},
    Builder, FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Gain, Integrator, Signal, Signals};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, GuideStar, OpticalModel, Processor, PyramidCalibrator,
    PyramidMeasurements, ResidualM2modes, WavefrontSensor, WavefrontStats,
};
use gmt_dos_clients_io::optics::{M2modes, SegmentPiston};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use interface::{units::NM, Data, Read, UniqueIdentifier, Update, Write, UID};
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
    let n_mode: usize = 450; //env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

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
        serde_pickle::from_reader(File::open(path.join("pymtor.pkl"))?, Default::default())?
    };
    let pym_constrained_recon: nalgebra::DMatrix<f32> = serde_pickle::from_reader(
        File::open(path.join("pym_constrained_recon.pkl"))?,
        Default::default(),
    )?;
    pymtor.set_hp_estimator(pym_constrained_recon);
    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    let pym = pym.build()?;
    let processor: Processor<_> = Processor::from(&pym);
    let pyramid = WavefrontSensor::<_, 1>::new(pym);
    let pym_ctrl = Integrator::<ResidualM2modes>::new(n_mode * 7).gain(0.5);

    let stats = WavefrontStats::<1>::default();

    // let piston: Signals = Signals::new(7, 40).channel(0, Signal::Constant(1000e-9));
    let mut rng = WyRand::new();
    let piston: Signals = (0..7).fold(Signals::new(7, 40), |signals, i| {
        signals.channel(
            i,
            Signal::Constant((2. * rng.generate::<f64>() - 1.) * 2.5e-6),
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

    let select = Select::<f64>::new((0..7).map(|i| i * n_mode).collect::<Vec<usize>>());
    let gain = Gain::new(vec![1e9; 7]);

    actorscript!(
        1: piston("Segment\nPiston")[SegmentPiston]
            -> &optical_model[GuideStar] -> stats[SegmentPiston<-9>] -> segment_piston_scope
        1: &optical_model[GuideStar]
            -> &pyramid[DetectorFrame<f32>]
                -> &processor[PyramidMeasurements]
                    -> calibrator("Reconstructor")[ResidualM2modes]
                        -> &pym_ctrl[M2modes]! -> &optical_model
        1: calibrator("Reconstructor")[ResidualM2modes] -> select("Reconstructed\nSegment\nPiston")[SegmentPistonRecon] ->gain[NM<SegmentPistonRecon>] -> segment_piston_recon_scope
    );

    monitor.await?;

    Ok(())
}

#[derive(UID)]
pub enum SegmentPistonRecon {}

pub enum Selection {
    Index(usize),
    Range(Range<usize>),
    Indices(Vec<usize>),
}

impl From<usize> for Selection {
    fn from(value: usize) -> Self {
        Self::Index(value)
    }
}

impl From<Range<usize>> for Selection {
    fn from(value: Range<usize>) -> Self {
        Self::Range(value)
    }
}

impl From<Vec<usize>> for Selection {
    fn from(value: Vec<usize>) -> Self {
        Self::Indices(value)
    }
}
pub struct Select<T> {
    selection: Selection,
    data: Arc<Vec<T>>,
    inner: PhantomData<T>,
}

impl<T> Select<T> {
    pub fn new(select: impl Into<Selection>) -> Self {
        Self {
            selection: select.into(),
            data: Arc::new(Vec::new()),
            inner: PhantomData,
        }
    }
}

impl<T: Send + Sync> Update for Select<T> {}

impl<T, U> Read<U> for Select<T>
where
    U: UniqueIdentifier<DataType = Vec<T>>,
    T: Send + Sync,
{
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}

impl<T, U> Write<U> for Select<T>
where
    U: UniqueIdentifier<DataType = Vec<T>>,
    T: Clone + Send + Sync,
{
    fn write(&mut self) -> Option<Data<U>> {
        match &self.selection {
            Selection::Index(idx) => self.data.get(*idx).map(|data| vec![data.clone()].into()),
            Selection::Range(range) => range
                .clone()
                .map(|idx| self.data.get(idx).cloned())
                .collect::<Option<Vec<T>>>()
                .map(|data| data.into()),
            Selection::Indices(idxs) => idxs
                .iter()
                .map(|idx| self.data.get(*idx).cloned())
                .collect::<Option<Vec<T>>>()
                .map(|data| data.into()),
        }
    }
}
