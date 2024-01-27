use std::{env, f64::consts::PI, fmt::Display, fs::File, path::Path, sync::Arc};

use crseo::{
    atmosphere,
    wavefrontsensor::{LensletArray, Pyramid},
    Atmosphere, Builder, FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Integrator, Timer};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, GuideStar, OpticalModel, Processor, PyramidCalibrator,
    PyramidMeasurements, ResidualM2modes, WavefrontSensor, WavefrontStats,
};
use gmt_dos_clients_io::optics::{M2modes, SegmentWfeRms, WfeRms};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use interface::{Data, Read, Tick, UniqueIdentifier, Update, Write, UID};

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
    // let (m2_modes, n_mode) = ("ASM_DDKLs_S7OC04184_675kls", 500);
    let (m2_modes, n_mode) = ("M2_OrthoNormGS36p_KarhunenLoeveModes", 500);

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
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2(m2_modes, n_mode),
        )
        .source(pym.guide_stars(None))
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    // Pyramid interaction matrix
    let file_name = format!("pymtor_{m2_modes}_{n_mode}.pkl");
    let path = data_repo.join(file_name);
    let mut pymtor: PyramidCalibrator = if !path.exists() {
        let pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, n_mode)
            .n_gpu(8)
            .build()?;
        serde_pickle::to_writer(&mut File::create(&path)?, &pymtor, Default::default())?;
        pymtor
    } else {
        println!("Loading {:?}", &path);
        serde_pickle::from_reader(File::open(&path)?, Default::default())?
    };
    assert_eq!(n_mode, pymtor.n_mode);
    println!("{pymtor}");

    let pym = pym.build()?;
    // Pyramid data processor
    let processor: Processor<_> = Processor::from(&pym);
    // Pyramid wafefront sensor
    let pyramid = WavefrontSensor::<_, 1>::new(pym);
    // Pyramid integral controller (modes>1)
    let pym_ctrl = Integrator::<ResidualM2modes>::new((n_mode - 1) * 7).gain(0.5);
    let stats = WavefrontStats::<1>::default();

    // Scopes definition
    let mut monitor = Monitor::new();
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let metronome: Timer = Timer::new(100);
    // Pyramid reconstructor
    pymtor.h00_estimator()?;
    let calibrator: Calibration<PyramidCalibrator> = pymtor.clone().into();

    actorscript!(
        #[model(name=closed_loop_pyramid_wo_piston, )]
        #[labels(metronome = "Metronome", calibrator = "Reconstructor", pym_ctrl="High-orders\nintegral controller")]
        1: metronome[Tick] -> optical_model[GuideStar] -> stats
        1: stats[SegmentWfeRms<-9>] -> segment_wfe_rms_scope
        1: stats[WfeRms<-9>] -> wfe_rms_scope
        1: optical_model[GuideStar]
            -> pyramid[DetectorFrame<f32>]
                -> processor[PyramidMeasurements] -> calibrator
        1: calibrator[ResidualM2modes]
            -> pym_ctrl[M2modes]!
                 -> optical_model
    );

    let optical_gain = OpticalGain::new(sampling_frequency as f64);
    let wait_for_it = WaitForIt::default();
    let metronome: Timer = Timer::new(1000);

    actorscript!(
        #[model(name=closed_loop_pyramid_wo_piston, )]
        #[labels(metronome = "Metronome", calibrator = "Reconstructor", pym_ctrl="High-orders\nintegral controller")]
        1: metronome[Tick] -> optical_model[GuideStar] -> stats
        1: stats[SegmentWfeRms<-9>] -> segment_wfe_rms_scope
        1: stats[WfeRms<-9>] -> wfe_rms_scope
        1: optical_model[GuideStar]
            -> pyramid[DetectorFrame<f32>]
                -> processor[PyramidMeasurements] -> calibrator

        1: calibrator[ResidualM2modes] -> wait_for_it[ResidualM2modes]! -> optical_gain
        1: calibrator[ResidualM2modes]
            -> pym_ctrl[M2modes]! -> optical_gain[M2modes]! -> optical_model
    );

    let mut og = optical_gain.lock().await;
    println!("{}", og.gain());

    monitor.await?;

    Ok(())
}

#[derive(Debug, Default)]
pub struct WaitForIt {
    data: Arc<Vec<f64>>,
    count: usize,
}

impl Update for WaitForIt {}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for WaitForIt {
    fn read(&mut self, data: Data<U>) {
        self.count += 1;
        if self.count > 1 {
            self.data = data.into_arc();
        }
    }
}
impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for WaitForIt {
    fn write(&mut self) -> Option<Data<U>> {
        Some((&self.data).into())
    }
}

#[derive(Debug, Default)]
pub struct Probe {
    sid: u8,
    mode: usize,
    n_mode: usize,
    amplitude: f64,
    frequency: f64,
    sampling_frequency: f64,
    signal: Vec<f64>,
    filtered: Vec<f64>,
    i: usize,
    gain: f64,
}

impl Display for Probe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            " * S{}#{:>3}({:.0}Hz): {:.3}",
            self.sid, self.mode, self.frequency, self.gain
        )
    }
}

fn variance(data: &[f64]) -> f64 {
    let n = data.len() as f64;
    let mean = data.iter().sum::<f64>() / n;
    let variance = data.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
    variance
}

impl Probe {
    pub fn id(&self) -> usize {
        (self.sid as usize - 1) * self.n_mode + self.mode
    }
    pub fn modulate(&mut self, signal: &mut f64) {
        let w = 2. * PI * self.frequency * self.i as f64 / self.sampling_frequency;
        let m = self.amplitude * w.sin();
        *signal += m;
        self.signal.push(*signal);
        self.i += 1
    }
    pub fn gain(&mut self) {
        let mut re_s = 0f64;
        let mut im_s = 0f64;
        let mut re_f = 0f64;
        let mut im_f = 0f64;
        for (i, (s, f)) in self.signal.iter().zip(&self.filtered).enumerate() {
            let w = 2. * PI * self.frequency * i as f64 / self.sampling_frequency;
            let (sin, cos) = w.sin_cos();
            re_s += s * cos;
            im_s += s * sin;
            re_f += f * cos;
            im_f += f * sin;
        }
        let d_s = re_s * re_s + im_s * im_s;
        let d_f = re_f * re_f + im_f * im_f;
        self.gain = (d_f / d_s).sqrt();
    }
    pub fn signal_variance(&self) -> f64 {
        variance(&self.signal)
    }
    pub fn filtered_variance(&self) -> f64 {
        variance(&self.filtered)
    }
    pub fn variance_ratio(&self) -> f64 {
        self.filtered_variance() / self.signal_variance()
    }
}

#[derive(Default)]
pub struct OpticalGain {
    probes: Vec<Probe>,
    data: Vec<f64>,
}

impl Display for OpticalGain {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Optical Gain:")?;
        self.probes.iter().map(|probe| probe.fmt(f)).collect()
    }
}
impl OpticalGain {
    pub fn new(sampling_frequency: f64) -> Self {
        let amplitude = 1e-8;
        let n_mode = 500;
        let outers = [1, 3, 5, 2, 4, 6]
            .into_iter()
            .zip([7, 40, 105, 192, 318, 460])
            .zip(vec![210.; 6]);
        let center = vec![7; 4]
            .into_iter()
            .zip([7, 105, 251, 401])
            .zip([80., 210., 310., 133.]);
        let probes: Vec<_> = outers
            .chain(center)
            .map(|((sid, mode), frequency)| Probe {
                sid,
                mode,
                n_mode,
                amplitude,
                frequency,
                sampling_frequency,
                ..Default::default()
            })
            .collect();
        Self {
            probes,
            data: vec![0f64; n_mode * 7],
        }
    }
    pub fn gain(&mut self) -> &mut Self {
        self.probes.iter_mut().for_each(Probe::gain);
        self
    }
    pub fn gain_from_variance(&self) -> Vec<f64> {
        self.probes
            .iter()
            .map(Probe::variance_ratio)
            .map(|x| (2. * x).sqrt())
            .collect()
    }
}

impl Update for OpticalGain {
    fn update(&mut self) {
        self.probes.iter_mut().for_each(|probe| {
            probe.modulate(&mut self.data[probe.id()]);
        });
    }
}

#[derive(UID)]
pub enum ProbingSignals {}

impl Write<M2modes> for OpticalGain {
    fn write(&mut self) -> Option<Data<M2modes>> {
        Some(self.data.clone().into())
    }
}

impl Read<M2modes> for OpticalGain {
    fn read(&mut self, data: Data<M2modes>) {
        self.data = data.into();
    }
}

impl Read<ResidualM2modes> for OpticalGain {
    fn read(&mut self, data: Data<ResidualM2modes>) {
        if !data.is_empty() {
            self.probes.iter_mut().for_each(|probe| {
                probe.filtered.push(data[probe.id()]);
            });
        }
    }
}
