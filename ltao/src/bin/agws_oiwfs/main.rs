use std::{fs::File, path::Path, sync::Arc};

use crseo::{
    gmt::{GmtM1, GmtM2},
    imaging::{Detector, LensletArray},
    Atmosphere, FromBuilder, Gmt, RayTracing, Source,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    fun::Fun,
    gif::Gif,
    operator::{Left, Operator, Right},
    print::Print,
    Gain, Integrator, Signals, Timer,
};
use gmt_dos_clients_crseo::{
    calibration::{Calibrate, CalibrationMode, Reconstructor},
    centroiding::{Full, ZeroMean},
    sensors::{
        Camera, DispersedFringeSensor, DispersedFringeSensorProcessing, NoSensor, WaveSensor,
        WaveSensorBuilder,
    },
    Centroids, DeviceInitialize, OpticalModel, OpticalModelBuilder,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, Frame, Host, SegmentPiston, SegmentWfeRms, Wavefront, WfeRms,
    },
};
use interface::{units::Mas, Data, Read, Tick, Units, Update, Write, UID};
use skyangle::Conversion;

const N_STEP: usize = 40;
const M2_N_MODE: usize = 66;
const OIWFS: usize = 1;

type LtwsCentroid = Centroids<Full>;
const LTWS_1ST_MODE: usize = 2;

const DFS_CAMERA_EXPOSURE: usize = 1;
const DFS_FFT_EXPOSURE: usize = 10;

type DFS = DispersedFringeSensor<DFS_CAMERA_EXPOSURE, DFS_FFT_EXPOSURE>;
type DFS11 = DispersedFringeSensor<1, 1>;
type DFSP11 = DispersedFringeSensorProcessing;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    std::env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("bin")
            .join("agws_oiwfs"),
    );

    let sampling_frequency = 1000f64;
    // let atm_builder = Atmosphere::builder().ray_tracing(
    //     RayTracing::default()
    //         .field_size(20f64.from_arcmin())
    //         .duration(30.)
    //         .filepath("dfs-atmosphere.bin"),
    // );

    let gmt_builder = Gmt::builder()
        .m2("Karhunen-Loeve", M2_N_MODE)
        .m1_truss_projection(false);

    // DispersedFringeSensor ...
    let agws_gs_builder = Source::builder().size(3).on_ring(6f32.from_arcmin());

    let mut dfs_om_builder = OpticalModel::<DFS>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone())
        // .atmosphere(atm_builder)
        .sensor(
            DFS::builder()
                .source(agws_gs_builder.clone())
                .nyquist_factor(3.),
        );
    let dfs_om = dfs_om_builder.clone().build()?;

    let offaxis_om: OpticalModel<WaveSensor> =
        OpticalModelBuilder::<WaveSensorBuilder>::from(&dfs_om_builder).build()?;
    println!("{offaxis_om}");
    // ... DispersedFringeSensor

    // OIWFS TIP-TILT SENSOR ...
    let oiwfs = Camera::builder().detector(Detector::default().n_px_imagelet(512));
    let mut oiwfs_centroids: Centroids = Centroids::try_from(&oiwfs)?;

    let mut oiwfs_tt_om_builder = OpticalModel::<Camera<OIWFS>>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("K"))
        .sensor(oiwfs);

    let mut calib_oiwfs_tt = <Centroids as Calibrate<GmtM2>>::calibrate(
        &((&oiwfs_tt_om_builder).into()),
        CalibrationMode::modes(M2_N_MODE, 1e-8) // .start_from(2)
            .ends_at(3),
    )?;
    calib_oiwfs_tt.pseudoinverse();
    println!("{calib_oiwfs_tt}");

    oiwfs_tt_om_builder.initialize(&mut oiwfs_centroids);
    dbg!(oiwfs_centroids.n_valid_lenslets());

    let oiwfs_tt_om = oiwfs_tt_om_builder
        // .atmosphere(atm_builder.clone())
        .build()?;
    println!("{oiwfs_tt_om}");
    // ... OIWFS TIP-TILT SENSOR

    // LTWS ...
    let ltws = Camera::builder()
        .lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(32))
        // .detector(Detector::default().n_px_framelet(10))
        .lenslet_flux(0.75);
    let mut ltws_centroids: LtwsCentroid = Centroids::try_from(&ltws)?;

    let mut ltws_om_builder = OpticalModel::<Camera<OIWFS>>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("V"))
        .sensor(ltws);

    let mut calib_m2_modes = <LtwsCentroid as Calibrate<GmtM2>>::calibrate(
        &((&ltws_om_builder).into()),
        CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(LTWS_1ST_MODE),
    )?;
    println!("{} cross-talks", calib_m2_modes.n_cross_talks());
    calib_m2_modes.pseudoinverse();
    println!("{calib_m2_modes}");
    serde_pickle::to_writer(
        &mut File::create("src/bin/agws_oiwfs/calib_m2_modes.pkl")?,
        &calib_m2_modes,
        Default::default(),
    )?;

    ltws_om_builder.initialize(&mut ltws_centroids);

    let ltws_om = ltws_om_builder
        //.atmosphere(atm_builder.clone())
        .build()?;
    println!("{ltws_om}");
    // ... LTWS

    let integrator = Integrator::new(M2_N_MODE * 7).gain(0.5);
    // let adder = Operator::new("+");

    let m1_rbm = Signals::new(42, 1000).channel(4 + 6, 1000f64.from_mas());

    let to_nm = Gain::new(vec![1e9; 6]);

    let oiwfs_gif = Gif::new("oiwfs.gif", 512, 512)?;
    // let ltws_gif = Gif::new("ltws.gif", 60 * 32, 60 * 32)?;
    let fun = Fun::new(|x: &Vec<f32>| x.iter().map(|x| x.cbrt()).collect::<Vec<f32>>());

    let timer: Timer = Timer::new(N_STEP / 2);
    let print = Print::default();
    actorscript!(
        #[model(name=agws_oiwfs)]
        #[labels(ltws_om="⤳ 🌫 ⤳\n⤳ GMT ⤳\n⤳ LTWS",
            integrator="Integrator",
            oiwfs_tt_om="⤳ 🌫 ⤳\n⤳ GMT ⤳\n⤳ OIWFS",
            calib_m2_modes = "M2 segment modes [2,200]\nreconstructor",
            calib_oiwfs_tt = "M2 segment modes [1,3]\nreconstructor",
            // adder="Add",
            fun = "∛",
            print="WFE RMS",
            m1_rbm="M1 RBM")]
        // LTWFS
        1: timer[Tick] -> ltws_om
        1: m1_rbm[M1RigidBodyMotions]
        -> ltws_om[Frame<Dev>]!
            -> ltws_centroids[LtwsData]
                -> calib_m2_modes[Left<LtwsResidualAsmCmd>]//${M2_N_MODE*7}
                    // -> adder[M2ASMAsmCommand]//${M2_N_MODE*7}
                        -> integrator[M2ASMAsmCommand]
                            -> ltws_om//[Frame<Host>].. -> ltws_gif
        1: ltws_om[WfeRms<-9>] -> print
        1: ltws_om[SegmentWfeRms<-9>] -> print
        1: ltws_om[Wavefront]$
        1: m1_rbm[M1RigidBodyMotions]
        // OIWFS
        -> oiwfs_tt_om[Frame<Dev>]!
            -> oiwfs_centroids[OiwfsData]
                -> calib_oiwfs_tt//[Right<OiwfsResidualAsmCmd>]//${M2_N_MODE*7}
                    // -> adder
        1: integrator[M2ASMAsmCommand] -> oiwfs_tt_om[Frame<Host>]$.. -> fun[Frame<Host>].. -> oiwfs_gif
        1: oiwfs_tt_om[OiwfsWavefront]$
        // DFS
        1: m1_rbm[M1RigidBodyMotions] -> offaxis_om
        1: integrator[M2ASMAsmCommand] -> offaxis_om[DfsWavefront]$
    );

    let mut dfs_processor = DispersedFringeSensorProcessing::new();
    dfs_om_builder.initialize(&mut dfs_processor);

    let dfs_recon = DfsReconstructor::new();
    let dfs_integrator = Integrator::new(42).gain(0.5);
    let add_m1_rbm = Operator::new("+");

    let oiwfs_gif = Gif::new("oiwfs_dfs.gif", 512, 512)?;

    let timer: Timer = Timer::new(120);
    let print_dfs = Print::default().tag("DFS");
    actorscript!(
        #[model(name=agws_oiwfs_dfs)]
        #[labels(ltws_om="⤳ 🌫 ⤳ GMT ⤳ LTWS",
            integrator="Integrator",
            oiwfs_tt_om="⤳ 🌫 ⤳ GMT ⤳ OIWFS",
            calib_m2_modes = "M2 segment modes [2,200]\nreconstructor",
            calib_oiwfs_tt = "M2 segment modes [1,3]\nreconstructor",
            // adder="Add",
            print="WFE RMS",m1_rbm="M1 RBM",
            dfs_om="⤳ 🌫 ⤳ GMT ⤳ DFS",
            add_m1_rbm="Add")]
        // LTWFS
        1: timer[Tick] -> ltws_om
        1: m1_rbm[Left<M1RigidBodyMotions>] -> add_m1_rbm[M1RigidBodyMotions]
        -> ltws_om[Frame<Dev>]!
            -> ltws_centroids[LtwsData]
                -> calib_m2_modes[Left<LtwsResidualAsmCmd>]//${M2_N_MODE*7}
                    // -> adder[M2ASMAsmCommand]//${M2_N_MODE*7}
                        -> integrator[M2ASMAsmCommand]
                            -> ltws_om
        1: ltws_om[WfeRms<-9>] -> print
        1: ltws_om[SegmentWfeRms<-9>] -> print
        1: ltws_om[Wavefront]$
        // OIWFS
        1: integrator[M2ASMAsmCommand] -> oiwfs_tt_om
        1: add_m1_rbm[M1RigidBodyMotions] -> oiwfs_tt_om
        // -> oiwfs_tt_om[Frame<Dev>]!
        //     -> oiwfs_centroids[OiwfsData]
        //         -> calib_oiwfs_tt[Right<OiwfsResidualAsmCmd>]//${M2_N_MODE*7}
        //             -> adder
        2: oiwfs_tt_om[Frame<Host>].. -> fun[Frame<Host>].. -> oiwfs_gif
        // DFS
        1: integrator[M2ASMAsmCommand] -> dfs_om
        1: add_m1_rbm[M1RigidBodyMotions] -> dfs_om
        10: dfs_om[DfsFftFrame<Dev>]! -> dfs_processor[Intercepts]
            -> dfs_recon[Mas<M1Rxy>] -> print_dfs
        10: dfs_recon[M1RigidBodyMotions] -> dfs_integrator[Right<M1RigidBodyMotions>] -> add_m1_rbm

        1: integrator[M2ASMAsmCommand] -> offaxis_om[DfsWavefront]$
        1: add_m1_rbm[M1RigidBodyMotions] -> offaxis_om
    );

    let mut logs = agws_oiwfs_dfs__logging_1.lock().await;
    println!("{}", logs);

    // logs.iter("M1RigidBodyMotions")?
    //     .last()
    //     .map(|x: Vec<f64>| x.into_iter().map(|x| x * 1e9).collect::<Vec<_>>())
    //     .map(|x: Vec<f64>| {
    //         dbg!(x);
    //     });

    // logs.iter("Left<M1RigidBodyMotions>")?
    //     .last()
    //     .map(|x: Vec<f64>| x.into_iter().map(|x| x * 1e9).collect::<Vec<_>>())
    //     .map(|x: Vec<f64>| {
    //         dbg!(x);
    //     });

    Ok(())
}

#[derive(UID)]
#[alias(name=Wavefront, client=OpticalModel<WaveSensor>, traits=Write,Size)]
pub enum DfsWavefront {}

#[derive(UID)]
#[alias(name=Wavefront, client=OpticalModel<Camera<1>>, traits=Write,Size)]
pub enum OiwfsWavefront {}

#[derive(UID)]
pub enum LtwsData {}

#[derive(UID)]
pub enum LtwsResidualAsmCmd {}

#[derive(UID)]
pub enum OiwfsResidualAsmCmd {}

#[derive(UID)]
pub enum OiwfsData {}

pub struct DfsReconstructor {
    recon: Reconstructor,
    intercepts: Arc<Vec<f64>>,
    rxy: Vec<f64>,
}
impl DfsReconstructor {
    pub fn new() -> Self {
        let recon: Reconstructor = serde_pickle::from_reader(
            File::open("src/bin/calibration/calib_dfs_closed-loop_m1-rxy.pkl").unwrap(),
            Default::default(),
        )
        .unwrap();
        Self {
            recon,
            intercepts: Default::default(),
            rxy: Default::default(),
        }
    }
}
impl Update for DfsReconstructor {
    fn update(&mut self) {
        let rxy = faer::mat::from_column_major_slice::<f64>(&self.intercepts, 36, 1) / &self.recon;
        self.rxy = rxy[0].col_as_slice(0).to_vec();
    }
}

impl Read<Intercepts> for DfsReconstructor {
    fn read(&mut self, data: Data<Intercepts>) {
        self.intercepts = data.into_arc();
    }
}

#[derive(UID)]
pub enum M1Rxy {}

impl Write<M1Rxy> for DfsReconstructor {
    fn write(&mut self) -> Option<Data<M1Rxy>> {
        Some(self.rxy.clone().into())
    }
}

impl Write<M1RigidBodyMotions> for DfsReconstructor {
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        let mut m1_rbm: Vec<f64> = self
            .rxy
            .chunks(2)
            .flat_map(|rxy| {
                vec![0.; 3]
                    .into_iter()
                    .chain(rxy.to_vec().into_iter())
                    .chain(Some(0.))
                    .collect::<Vec<_>>()
            })
            .collect();
        m1_rbm.extend(vec![0f64; 6]);
        Some(m1_rbm.into())
    }
}

impl Units for DfsReconstructor {}
