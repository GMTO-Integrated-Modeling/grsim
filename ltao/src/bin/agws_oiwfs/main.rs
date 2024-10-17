use std::{fs::File, path::Path, sync::Arc};

use crseo::{
    gmt::GmtM2,
    imaging::{Detector, LensletArray},
    FromBuilder, Gmt, Source,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    fun::Fun,
    gif::Gif,
    operator::{Left, Operator, Right},
    Integrator, Signals, Timer,
};
use gmt_dos_clients_crseo::{
    calibration::{
        algebra::Collapse, Calibrate, CalibrationMode, ClosedLoopCalib, ClosedLoopCalibrate,
        ClosedLoopReconstructor, MirrorMode, Reconstructor,
    },
    centroiding::{CentroidsProcessing, ZeroMean},
    sensors::{builders::WaveSensorBuilder, Camera, DispersedFringeSensor, WaveSensor},
    DeviceInitialize, DispersedFringeSensorProcessing, OpticalModel, OpticalModelBuilder,
};
use gmt_dos_clients_io::{
    gmt_m1::{M1ModeShapes, M1RigidBodyMotions},
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, Frame, Host, SegmentWfeRms, Wavefront, WfeRms,
    },
};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use grsim_ltao::{
    agws_reconstructor::{Disjoined, Merged},
    AgwsReconstructor, DfsWavefront, LtwsData, LtwsResidualAsmCmd, M1ModesNorm, M1Rxy, OiwfsData,
    OiwfsResidualAsmCmd, Sh48Data,
};
use interface::{Data, Read, Tick, Units, Update};
use nanorand::{Rng, WyRand};
use skyangle::Conversion;

const N_STEP: usize = 40;
const M1_N_MODE: usize = 27;
const M2_N_MODE: usize = 66;
const OIWFS: usize = 1;
const AGWS_N_GS: usize = 3;

type LtwsCentroid = CentroidsProcessing<ZeroMean>;
const LTWS_1ST_MODE: usize = 2;

const DFS_CAMERA_EXPOSURE: usize = 1;
const DFS_FFT_EXPOSURE: usize = 10;

type DFS = DispersedFringeSensor<DFS_CAMERA_EXPOSURE, DFS_FFT_EXPOSURE>;
// type DFS11 = DispersedFringeSensor<1, 1>;
// type DFSP11 = DispersedFringeSensorProcessing;

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
        .m1("bending modes", M1_N_MODE)
        .m2("Karhunen-Loeve", M2_N_MODE)
        .m1_truss_projection(false);

    // DispersedFringeSensor ...
    let agws_gs_builder = Source::builder()
        .size(AGWS_N_GS)
        .on_ring(6f32.from_arcmin());

    let dfs_om_builder = OpticalModel::<DFS>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone().band("J"))
        // .atmosphere(atm_builder)
        .sensor(DFS::builder().source(agws_gs_builder.clone().band("J")));
    let dfs_om = dfs_om_builder.clone().build()?;
    println!("{dfs_om}");

    let offaxis_om: OpticalModel<WaveSensor> =
        OpticalModelBuilder::<WaveSensorBuilder>::from(&dfs_om_builder).build()?;
    println!("{offaxis_om}");
    // ... DispersedFringeSensor

    // AGWS SH48 ...
    let sh48 = Camera::builder()
        .n_sensor(AGWS_N_GS)
        .lenslet_array(
            LensletArray::default()
                .n_side_lenslet(48)
                .n_px_lenslet(24)
                .pitch(0.53),
        )
        .detector(Detector::default().n_px_framelet(8))
        .lenslet_flux(0.75);
    let mut sh48_centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sh48)?;

    let sh48_om_builder = OpticalModel::<Camera<1>>::builder()
        .gmt(gmt_builder.clone())
        .source(
            agws_gs_builder
                .pupil_size(48 as f64 * 0.53)
                .band("R")
                .fwhm(6.),
        )
        .sensor(sh48);
    let mut file = File::create("src/bin/agws_oiwfs/sh48.ron")?;
    ron::ser::to_writer_pretty(&mut file, &sh48_om_builder, Default::default())?;

    sh48_om_builder.initialize(&mut sh48_centroids);
    dbg!(sh48_centroids.n_valid_lenslets());

    let calib_sh48_bm: ClosedLoopReconstructor =
        if let Ok(file) = File::open(format!("src/bin/agws_oiwfs/calib_sh48_{M1_N_MODE}bm.pkl")) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let closed_loop_optical_model =
                OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone());
            let mut calib_sh48_bm =
                <CentroidsProcessing as ClosedLoopCalibrate<WaveSensor>>::calibrate(
                    &sh48_om_builder.clone().into(),
                    CalibrationMode::modes(M1_N_MODE, 1e-4),
                    &closed_loop_optical_model,
                    CalibrationMode::modes(M2_N_MODE, 1e-6).start_from(2),
                )?;
            calib_sh48_bm.pseudoinverse();
            serde_pickle::to_writer(
                &mut File::create(format!("src/bin/agws_oiwfs/calib_sh48_{M1_N_MODE}bm.pkl"))?,
                &calib_sh48_bm,
                Default::default(),
            )?;
            calib_sh48_bm
        };
    println!("{calib_sh48_bm}");

    let sh48_om = sh48_om_builder.build()?;
    println!("{sh48_om}");
    // ... AGWS SH48

    // OIWFS TIP-TILT SENSOR ...
    let oiwfs = Camera::builder().detector(Detector::default().n_px_imagelet(512));
    let mut oiwfs_centroids: CentroidsProcessing = CentroidsProcessing::try_from(&oiwfs)?;

    let oiwfs_tt_om_builder = OpticalModel::<Camera<OIWFS>>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("K"))
        .sensor(oiwfs);

    let calib_oiwfs_tt: Reconstructor =
        if let Ok(file) = File::open("src/bin/agws_oiwfs/calib_oiwfs_tt.pkl") {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let mut calib_oiwfs_tt = <CentroidsProcessing as Calibrate<GmtM2>>::calibrate(
                &((&oiwfs_tt_om_builder).into()),
                CalibrationMode::modes(M2_N_MODE, 1e-8) // .start_from(2)
                    .ends_at(3),
            )?;
            calib_oiwfs_tt.pseudoinverse();
            serde_pickle::to_writer(
                &mut File::create("src/bin/agws_oiwfs/calib_oiwfs_tt.pkl")?,
                &calib_oiwfs_tt,
                Default::default(),
            )?;
            calib_oiwfs_tt
        };
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
    let mut ltws_centroids: LtwsCentroid = CentroidsProcessing::try_from(&ltws)?;

    let ltws_om_builder = OpticalModel::<Camera<OIWFS>>::builder()
        .sampling_frequency(sampling_frequency)
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("V"))
        .sensor(ltws);

    let calib_m2_modes: Reconstructor =
        if let Ok(file) = File::open("src/bin/agws_oiwfs/calib_m2_modes.pkl") {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let mut calib_m2_modes = <LtwsCentroid as Calibrate<GmtM2>>::calibrate(
                &((&ltws_om_builder).into()),
                CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(LTWS_1ST_MODE),
            )?;
            println!("{} cross-talks", calib_m2_modes.n_cross_talks());
            calib_m2_modes.pseudoinverse();
            serde_pickle::to_writer(
                &mut File::create("src/bin/agws_oiwfs/calib_m2_modes.pkl")?,
                &calib_m2_modes,
                Default::default(),
            )?;
            calib_m2_modes
        };

    println!("{calib_m2_modes}");

    ltws_om_builder.initialize(&mut ltws_centroids);
    dbg!(ltws_centroids.n_valid_lenslets());

    let ltws_om = ltws_om_builder
        //.atmosphere(atm_builder.clone())
        .build()?;
    println!("{ltws_om}");
    // ... LTWS

    let integrator = Integrator::new(M2_N_MODE * 7).gain(0.5);
    let adder = Operator::new("+");

    /*     let mut rng = WyRand::new();

    let m1_rbm = (0..7).fold(Signals::new(42, 1000), |s, i| {
        s.channel(
            3 + 6 * i,
            (2. * rng.generate::<f64>() - 1.) * 250f64.from_mas(),
        )
        .channel(
            4 + 6 * 1,
            (2. * rng.generate::<f64>() - 1.) * 2520f64.from_mas(),
        )
    });
    let m1_bm = (0..7).fold(Signals::new(M1_N_MODE * 7, 1000), |s, i| {
        (0..11).fold(s, |s, j| {
            s.channel(
                M1_N_MODE * i + j,
                (2. * rng.generate::<f64>() - 1.) * 1e-4 / ((j + 1) as f64),
            )
        })
    }); */

    let m1_rbm = Signals::new(42, 1000).channel(6 * 0 + 3, 100f64.from_mas());
    let m1_bm = Signals::new(M1_N_MODE * 7, 1000);
    // Signals::new(M1_N_MODE * 7, 1000).channel(0, 1e-4);

    // let to_nm = Gain::new(vec![1e9; 6]);

    let oiwfs_gif = Gif::new("oiwfs.gif", 512, 512)?;
    // let ltws_gif = Gif::new("ltws.gif", 60 * 32, 60 * 32)?;
    let fun = Fun::new(|x: &Vec<f32>| x.iter().map(|x| x.cbrt()).collect::<Vec<f32>>());

    // SCOPES
    let mut monitor = Monitor::new();
    let segment_wfe_rms_scope = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let wfe_rms_scope = Scope::<WfeRms<-9>>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let m1_bm_scope = Scope::<M1ModesNorm>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let m1_rxy_scope = Scope::<M1Rxy>::builder(&mut monitor)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;

    let timer: Timer = Timer::new(N_STEP / 2);

    let m1_bm_norm = Fun::new(|x: &Vec<f64>| {
        x.chunks(M1_N_MODE)
            .map(|x| (x.iter().map(|x| x * x).sum::<f64>() / M1_N_MODE as f64).sqrt())
            .collect::<Vec<f64>>()
    });
    let m1_rxy_norm = Fun::new(|x: &Vec<f64>| {
        x.chunks(6)
            .map(|x| x[3].hypot(x[4]).to_mas())
            .collect::<Vec<f64>>()
    });
    // let print = Print::default();
    actorscript!(
        #[model(name=agws_oiwfs)]
        #[labels(ltws_om="⤳ GMT ⤳\n⤳ LTWS",//"⤳ 🌫 ⤳\n⤳ GMT ⤳\n⤳ LTWS"
            integrator="Integrator",
            oiwfs_tt_om="⤳ GMT ⤳\n⤳ OIWFS",
            calib_m2_modes = "M2 segment modes [2,200]\nreconstructor",
            calib_oiwfs_tt = "M2 segment modes [1,3]\nreconstructor",
            adder="Add",
            offaxis_om = "⤳ GMT ⤳\n⤳ DFS GSs",
            fun = "∛",
            // print="WFE RMS",
            m1_rbm="M1 RBM",
            m1_bm="M1 BM")]
        // LTWFS
        1: timer[Tick] -> ltws_om
        1: m1_bm[M1ModeShapes] -> m1_bm_norm[M1ModesNorm] -> m1_bm_scope
        1: m1_rbm[M1RigidBodyMotions] -> m1_rxy_norm[M1Rxy] -> m1_rxy_scope
        1: m1_bm[M1ModeShapes] -> ltws_om
        1: m1_rbm[M1RigidBodyMotions] -> ltws_om[Frame<Dev>]!
            -> ltws_centroids[LtwsData]
                -> calib_m2_modes[Left<LtwsResidualAsmCmd>]//${M2_N_MODE*7}
                    -> adder[M2ASMAsmCommand]//${M2_N_MODE*7}
                        -> integrator[M2ASMAsmCommand]
                            -> ltws_om//[Frame<Host>].. -> ltws_gif
        1: ltws_om[WfeRms<-9>].. -> wfe_rms_scope
        1: ltws_om[SegmentWfeRms<-9>].. -> segment_wfe_rms_scope
        10: ltws_om[Wavefront]${1921*1921}
        // OIWFS
        1: m1_bm[M1ModeShapes] -> oiwfs_tt_om
        1: m1_rbm[M1RigidBodyMotions]
        -> oiwfs_tt_om[Frame<Dev>]!
           -> oiwfs_centroids[OiwfsData]
                -> calib_oiwfs_tt[Right<OiwfsResidualAsmCmd>]//${M2_N_MODE*7}
                    -> adder
        1: integrator[M2ASMAsmCommand] -> oiwfs_tt_om[Frame<Host>].. -> fun[Frame<Host>].. -> oiwfs_gif
        // 1: oiwfs_tt_om[OiwfsWavefront]$
        // DFS
        1: m1_bm[M1ModeShapes] -> offaxis_om
        1: m1_rbm[M1RigidBodyMotions] -> offaxis_om
        1: integrator[M2ASMAsmCommand] -> offaxis_om
        10: offaxis_om[DfsWavefront]$
    );

    let mut dfs_processor = DispersedFringeSensorProcessing::new();
    dfs_om_builder.initialize(&mut dfs_processor);

    // let (m1_to_m2, mut recon) =
    //     <DispersedFringeSensorProcessing as ClosedLoopCalibrate>::calibrate(
    //         dfs_om_builder.clone_into::<1, 1>(),
    //         CalibrationMode::RBM([
    //             None,                     // Tx
    //             None,                     // Ty
    //             None,                     // Tz
    //             Some(1f64.from_arcsec()), // Rx
    //             Some(1f64.from_arcsec()), // Ry
    //             None,                     // Rz
    //         ]),
    //         OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone()),
    //         CalibrationMode::modes(M2_N_MODE, 1e-6),
    //     )?;
    // recon.pseudoinverse();
    // println!("{recon}");

    // let dfs_recon =
    //     DfsReconstructor::new("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-rxy_v2.pkl");
    // let dfs_recon =
    //     DfsReconstructor::new("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-21bm.pkl");
    let agws_recon = AgwsReconstructor::<Disjoined>::new(M1_N_MODE)?;

    let dfs_integrator = Integrator::new(42).gain(0.5);
    let sh48_integrator = Integrator::new(M1_N_MODE * 7).gain(0.5);
    let add_m1_rbm = Operator::new("+");
    let add_m1_bm = Operator::new("+");

    let oiwfs_gif = Gif::new("oiwfs_dfs.gif", 512, 512)?;

    // let timer: Timer = Timer::new(120);
    // let print_dfs = Print::default().tag("DFS");
    actorscript!(
        #[model(name=agws_oiwfs_dfs)]
        #[labels(ltws_om="⤳ GMT ⤳\n⤳ LTWS",
            integrator="ASM modes\nintegrator",
            dfs_integrator="M1 RBM\nintegrator",
            sh48_integrator="M1 modes\nintegrator",
            oiwfs_tt_om="⤳ GMT ⤳\n⤳ OIWFS",
            calib_m2_modes = "LTWFS\n -> M2 modes",
            calib_oiwfs_tt = "OIWFS\n -> M2 modes [1,3]",
            offaxis_om = "⤳ GMT ⤳\n⤳ DFS GSs",
            adder="Add",
            // print="WFE RMS",m1_rbm="M1 RBM",
            sh48_om="⤳ GMT ⤳\n⤳ SH48",
            // calib_sh48_bm="SH48\n -> M1 bending modes",
            dfs_om="⤳ GMT ⤳\n⤳ DFS",
            add_m1_rbm="Add M1 RBMs",
            add_m1_bm="Add M1 BMs",
            m1_bm="M1 BM",
            agws_recon="SH48 + DFS\n -> M1 RBM\n -> M1 BM",
            dfs_processor="DFS fftlets\nprocessing",
            wfe_rms_scope="\u{1F4DF}",segment_wfe_rms_scope="\u{1F4DF}")]
        // LTWFS
        // 1: timer[Tick] -> ltws_om
        1: m1_bm[Left<M1ModeShapes>] -> add_m1_bm[M1ModeShapes] -> ltws_om
        1: add_m1_bm[M1ModeShapes] -> m1_bm_norm[M1ModesNorm] -> m1_bm_scope
        1: add_m1_rbm[M1RigidBodyMotions] -> m1_rxy_norm[M1Rxy] -> m1_rxy_scope
        1: m1_rbm[Left<M1RigidBodyMotions>] -> add_m1_rbm[M1RigidBodyMotions]
        -> ltws_om[Frame<Dev>]!
            -> ltws_centroids[LtwsData]
                -> calib_m2_modes[Left<LtwsResidualAsmCmd>]//${M2_N_MODE*7}
                    -> adder[M2ASMAsmCommand]//${M2_N_MODE*7}
                        -> integrator[M2ASMAsmCommand]
                            -> ltws_om
        1: ltws_om[WfeRms<-9>].. -> wfe_rms_scope
        1: ltws_om[SegmentWfeRms<-9>].. -> segment_wfe_rms_scope
        10: ltws_om[Wavefront]${1921*1921}
        // OIWFS
        1: integrator[M2ASMAsmCommand] -> oiwfs_tt_om
        1: add_m1_bm[M1ModeShapes] -> oiwfs_tt_om
        1: add_m1_rbm[M1RigidBodyMotions]
        -> oiwfs_tt_om[Frame<Dev>]!
            -> oiwfs_centroids[OiwfsData]
                -> calib_oiwfs_tt[Right<OiwfsResidualAsmCmd>]//${M2_N_MODE*7}
                    -> adder
        1: oiwfs_tt_om[Frame<Host>].. -> fun[Frame<Host>].. -> oiwfs_gif
        // DFS
        1: integrator[M2ASMAsmCommand] -> dfs_om
        1: add_m1_bm[M1ModeShapes] -> dfs_om
        1: add_m1_rbm[M1RigidBodyMotions] -> dfs_om
        10: dfs_om[DfsFftFrame<Dev>]! -> dfs_processor[Intercepts] -> agws_recon
            //  -> dfs_recon[Mas<M1Rxy>] -> print_dfs
        10: agws_recon[M1RigidBodyMotions] -> dfs_integrator[Right<M1RigidBodyMotions>] -> add_m1_rbm
        // SH48
        1: integrator[M2ASMAsmCommand] -> sh48_om
        1: add_m1_rbm[M1RigidBodyMotions] -> sh48_om
        1: add_m1_bm[M1ModeShapes] -> sh48_om
        10: sh48_om[Frame<Dev>]!
            -> sh48_centroids[Sh48Data]
                -> agws_recon[M1ModeShapes]
                    -> sh48_integrator[Right<M1ModeShapes>]
                        -> add_m1_bm

        1: integrator[M2ASMAsmCommand] -> offaxis_om
        1: add_m1_bm[M1ModeShapes] -> offaxis_om
        1: add_m1_rbm[M1RigidBodyMotions] -> offaxis_om
        10: offaxis_om[DfsWavefront]$
    );

    let mut logs = agws_oiwfs_dfs_logging_10.lock().await;
    println!("{}", logs);
    agws_oiwfs_logging_10
        .lock()
        .await
        .to_parquet("agws_oiwfs_data_10.parquet")?;
    logs.to_parquet("agws_oiwfs_dfs_data_10.parquet")?;
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

    monitor.await?;

    Ok(())
}

pub struct DfsReconstructor {
    recon: Reconstructor<MirrorMode>,
    intercepts: Arc<Vec<f64>>,
    rxy: Vec<f64>,
}
impl From<Reconstructor<MirrorMode>> for DfsReconstructor {
    fn from(recon: Reconstructor<MirrorMode>) -> Self {
        Self {
            recon,
            intercepts: Default::default(),
            rxy: Default::default(),
        }
    }
}
impl DfsReconstructor {
    pub fn new<P: AsRef<Path>>(path: P) -> Self {
        let recon: Reconstructor<CalibrationMode, ClosedLoopCalib<CalibrationMode>> =
            serde_pickle::from_reader(File::open(path.as_ref()).unwrap(), Default::default())
                .unwrap();
        let mut recon = recon.collapse();
        recon.pseudoinverse();
        println!("{recon}");
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

impl interface::Write<M1Rxy> for DfsReconstructor {
    fn write(&mut self) -> Option<Data<M1Rxy>> {
        Some(self.rxy.clone().into())
    }
}

impl interface::Write<M1RigidBodyMotions> for DfsReconstructor {
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

impl interface::Write<M1ModeShapes> for DfsReconstructor {
    fn write(&mut self) -> Option<Data<M1ModeShapes>> {
        let mut m1_bm = self.rxy.clone();
        m1_bm.extend(vec![0f64; M1_N_MODE]);
        Some(m1_bm.into())
    }
}

impl Units for DfsReconstructor {}
