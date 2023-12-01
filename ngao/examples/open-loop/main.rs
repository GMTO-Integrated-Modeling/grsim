use std::fs::File;
use std::path::Path;
use std::sync::Arc;
use std::{env, mem};

use crseo::wavefrontsensor::{LensletArray, Pyramid};
use crseo::{atmosphere, Atmosphere, Builder, FromBuilder, Gmt, WavefrontSensorBuilder};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{once::Once, Average, Integrator, Timer};
use gmt_dos_clients_crseo::{
    DetectorFrame, GuideStar, OpticalModel, ResidualM2modes, WavefrontSensor,
};
use gmt_dos_clients_io::optics::{
    M2modes, SegmentD7Piston, SegmentDWfe, SegmentPiston, SegmentWfe, SegmentWfeRms, WfeRms,
};
use grsim_ngao::{Calibration, Processor, PyramidCalibrator, PyramidMeasurements};
// use interface::units::NM;
use interface::{Data, Read, Size, Tick, UniqueIdentifier, Update, Write, UID};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );

    let n_lenslet = 92;
    let m2_modes = "ASM_DDKLs_S7OC04184_675kls";
    let n_mode: usize = 450; //env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(5f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );

    let sampling_frequency = 1_000usize; // Hz

    let optical_model = OpticalModel::builder()
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .source(Default::default())
        .atmosphere(atm_builder)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    // let pym = pym.build()?;
    // let processor: Processor<_> = Processor::from(&pym);
    // let pyramid = WavefrontSensor::<_, 1>::new(pym);
    // let pym_ctrl = Integrator::<ResidualM2modes>::new(n_mode * 7).gain(0.5);

    let n_sample = 100;
    let metronome: Timer = Timer::new(n_sample);
    let stats = WavefrontStats::default();
    // let act_print = Print::<_>::default();
    // let act_print32 = Print::<f32>::default();
    // type WfeRmsNm = NM<WfeRms>;

    /*     actorscript! {
        #[model(name = highorder, state = completed)]
        1: metronome[Tick]
            -> &optical_model[GuideStar] -> stats[WfeRms]$
    }; */

    use ::gmt_dos_actors::{
        framework::{
            model::FlowChart,
            network::{AddActorOutput, AddOuput, IntoLogs, IntoLogsN, TryIntoInputs},
        },
        model, ArcMutex,
    };
    let mut stats: ::gmt_dos_actors::prelude::Actor<_, 1, 1> = stats.into();
    let mut metronome: ::gmt_dos_actors::prelude::Actor<_, 0, 1> = metronome.into();
    let mut logging_1 = ::gmt_dos_clients_arrow::Arrow::builder(1000)
        .filename("data_1")
        .build()
        .into_arcx();
    let mut data_1: ::gmt_dos_actors::prelude::Actor<_, 1, 0> =
        ::gmt_dos_actors::prelude::Actor::new(logging_1.clone()).name("data_1");
    let optical_model = optical_model.into_arcx();
    let mut optical_model_actor: ::gmt_dos_actors::prelude::Actor<_, 1, 1> =
        ::gmt_dos_actors::prelude::Actor::new(optical_model.clone());
    metronome
        .add_output()
        .build::<Tick>()
        .into_input(&mut optical_model_actor)?;
    /*     optical_model_actor
    .add_output()
    .build::<GuideStar>()
    .into_input(&mut stats)?; */
    optical_model_actor
        .add_output()
        .unbounded()
        .build::<WfeRms>()
        .log(&mut data_1)
        .await?;
    #[allow(unused_variables)]
    let highorder = model!(metronome, data_1, optical_model_actor)
        .name("highorder")
        .flowchart()
        .check()?
        .run()
        .await?;

    println!("{}", logging_1.lock().await);

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

impl Read<MayBeSegmentAD7Piston> for Merger {
    fn read(&mut self, data: Data<MayBeSegmentAD7Piston>) {
        self.hdfs = data.into_arc();
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
            let piston = self.pym.chunks(self.n_mode).map(|x| x[0]).sum::<f64>() / 7f64;
            self.pym
                .chunks_mut(self.n_mode)
                .for_each(|x| x[0] -= piston);
        }
        Some(mem::take(&mut self.pym).into())
    }
}

#[derive(UID)]
pub enum SegmentAD7Piston {}
pub enum MayBeSegmentAD7Piston {}
impl UniqueIdentifier for MayBeSegmentAD7Piston {
    type DataType = Option<Vec<f64>>;
}

#[derive(Debug, Default)]
pub struct WavefrontStats<const N_SRC: usize = 1> {
    segment_wfe: Vec<(f64, f64)>,
    wfe_rms: Vec<f64>,
}

impl<const N_SRC: usize> Update for WavefrontStats<N_SRC> {}

impl<const N_SRC: usize> Read<GuideStar> for WavefrontStats<N_SRC> {
    fn read(&mut self, data: Data<GuideStar>) {
        let src = &mut (data.lock().unwrap());
        self.wfe_rms = src.wfe_rms();
        self.segment_wfe = src.segment_wfe();
    }
}

impl<const N_SRC: usize, const E: i32> Size<WfeRms<E>> for WavefrontStats<N_SRC> {
    fn len(&self) -> usize {
        N_SRC
    }
}

impl<const N_SRC: usize, const E: i32> Write<WfeRms<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<WfeRms<E>>> {
        let data = &self.wfe_rms;
        Some(
            data.into_iter()
                .map(|s| *s * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

pub enum Wfe<const E: i32 = 0> {}
impl<const E: i32> UniqueIdentifier for Wfe<E> {
    type DataType = (Vec<f64>, Vec<(f64, f64)>);
}
impl<const N_SRC: usize, const E: i32> Write<Wfe<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<Wfe<E>>> {
        let data: Vec<_> = self
            .segment_wfe
            .iter()
            .map(|(p, s)| (*p * 10_f64.powi(-E), *s * 10_f64.powi(-E)))
            .collect();
        let wfe_rms: Vec<_> = self.wfe_rms.iter().map(|x| x * 10_f64.powi(-E)).collect();
        Some(Data::new((wfe_rms, data)))
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentWfe<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentWfe<E>>> {
        let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(p, s)| (*p * 10_f64.powi(-E), *s * 10_f64.powi(-E)))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentDWfe<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentDWfe<E>>> {
        let data = &self.segment_wfe;
        let p7 = data[6].0;
        Some(
            data.into_iter()
                .map(|(p, s)| ((*p - p7) * 10_f64.powi(-E), *s * 10_f64.powi(-E)))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<const N_SRC: usize, const E: i32> Size<SegmentPiston<E>> for WavefrontStats<N_SRC> {
    fn len(&self) -> usize {
        N_SRC * 7
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentPiston<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentPiston<E>>> {
        let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(p, _)| *p * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentD7Piston<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentD7Piston<E>>> {
        let data = &self.segment_wfe;
        let p7 = data[6].0;
        let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(p, _)| (*p - p7) * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<const N_SRC: usize, const E: i32> Size<SegmentWfeRms<E>> for WavefrontStats<N_SRC> {
    fn len(&self) -> usize {
        N_SRC * 7
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentWfeRms<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentWfeRms<E>>> {
        let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(_, s)| *s * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}
