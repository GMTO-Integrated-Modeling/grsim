//
//
//

/*
FEM_REPO=`pwd`/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/ cargo run --release --features modal_asm_cmd
*/

use std::{env, error::Error, mem, path::Path, sync::Arc};

use gmt_dos_actors::{actorscript, system::Sys};
//use gmt_dos_clients::Weight;
use crseo::{
    atmosphere,
    wavefrontsensor::{LensletArray, Pyramid},
    Atmosphere, FromBuilder, Gmt, WavefrontSensorBuilder,
};
use gmt_dos_clients::{print::Print, Integrator, Timer};
use gmt_dos_clients_crseo::{
    Calibration, DetectorFrame, OpticalModel, Processor, PyramidCalibrator, PyramidMeasurements,
    ResidualM2modes,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{
        asm::{M2ASMAsmCommand, M2ASMFaceSheetFigure, M2ASMReferenceBodyNodes},
        M2RigidBodyMotions,
    },
    optics::{M2modes, SegmentWfeRms, Wavefront, WfeRms},
};
use gmt_dos_clients_servos::{
    asms_servo::{self, ReferenceBody},
    AsmsServo, GmtM2, GmtServoMechanisms,
};
use gmt_fem::FEM;
use interface::{filing::Filing, units::MuM, Data, Read, Size, Tick, Update, Write};
use matio_rs::{MatFile, MatioError};
use nalgebra as na;

const ACTUATOR_RATE: usize = 80;

#[derive(Debug, Clone)]
struct MyFacesheet;
impl asms_servo::FacesheetOptions for MyFacesheet {
    fn remove_rigid_body_motions(&self) -> bool {
        false
    }
}

const N_MODE: usize = 500;
const N_ACTUATOR: usize = 675;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);

    let sim_sampling_frequency = 8000;
    let optical_model_sampling_frequency = 1000;
    //let sim_duration = 5_usize; // second
    let n_step = 100; //sim_sampling_frequency * sim_duration;

    // M2 modal basis
    let m2_modes = "M2_OrthoNormGS36p90_KarhunenLoeveModes";

    // Pyramid definition
    let n_lenslet = 92;
    let pym = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);

    let calibrator: Calibration<PyramidCalibrator> = {
        let filename = format!("{0}/pym-{m2_modes}-{N_MODE}.bin", env!("GMT_MODES_PATH"));
        if let Ok(pymtor) = PyramidCalibrator::try_from(filename.as_str()) {
            pymtor
        } else {
            let mut pymtor = PyramidCalibrator::builder(pym.clone(), m2_modes, N_MODE)
                .n_thread(7)
                .build()?;
            pymtor.h00_estimator()?;
            pymtor.save(filename)?
        }
        .into()
    };
    let processor: Processor<_> = Processor::try_from(&pym)?;
    let pym_ctrl: Integrator<ResidualM2modes> =
        Integrator::<ResidualM2modes>::new(N_MODE * 7).gain(0.5);

    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(5f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );
    let optical_model = OpticalModel::<Pyramid>::builder()
        .gmt(
            Gmt::builder()
                .m1_truss_projection(false)
                .m2("ASM_IFs_permutated_90", N_ACTUATOR),
        )
        .source(pym.guide_stars(None))
        .sensor(pym.clone())
        .atmosphere(atm_builder)
        .sampling_frequency(optical_model_sampling_frequency as f64)
        .build()?;

    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path_or_else(
        Path::new(env!("FEM_REPO")).join("servos.bin"),
        || {
            GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(
                sim_sampling_frequency as f64,
                FEM::from_env().unwrap(),
            )
            //.wind_loads(WindLoads::new())
            .asms_servo(
                AsmsServo::new()
                    .facesheet(Default::default())
                    .reference_body(ReferenceBody::new()),
            )
        },
    )?;

    let modes2actuators = ModalToZonal::new().unwrap();

    let metronome: Timer = Timer::new(n_step);
    let prt = Print::default();

    actorscript! (
        8: metronome[Tick]
            -> optical_model[DetectorFrame]
                 -> processor[PyramidMeasurements]
                     -> calibrator[ResidualM2modes]
                         -> pym_ctrl[M2modes]!
                            -> modes2actuators

        1: modes2actuators[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        1: {gmt_servos::GmtFem}[M2ASMFaceSheetFigure] -> optical_model

        8: optical_model[WfeRms<-9>]$.. -> prt
        8: optical_model[SegmentWfeRms<-9>]$.. -> prt

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> optical_model
        1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes] -> optical_model
    );

    // let metronome: Timer = Timer::new(0);
    // actorscript!(
    //     #[model(name=wavefront)]
    //     1: metronome[Tick] -> optical_model[Wavefront]!$
    // );

    let mut opm = optical_model.lock().await;
    let phase = <OpticalModel<Pyramid> as Write<MuM<Wavefront>>>::write(&mut opm).unwrap();
    let n_px = (<OpticalModel<Pyramid> as Size<Wavefront>>::len(&mut opm) as f64).sqrt() as usize;

    let _: complot::Heatmap = ((phase.as_arc().as_slice(), (n_px, n_px)), None).into();

    Ok(())
}

#[derive(Debug)]
pub struct ModalToZonal {
    mats: Vec<na::DMatrix<f64>>,
    modes: Arc<Vec<f64>>,
    actuators: Vec<f64>,
}

impl ModalToZonal {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set!");
        let fem_path = Path::new(&fem_var);
        let mat_file = MatFile::load(&fem_path.join("KLmodesGS36p90.mat"))?;
        Ok(Self {
            mats: (1..=7)
                .map(|i| mat_file.var::<_, na::DMatrix<f64>>(format!("KL_{i}")))
                .collect::<Result<Vec<na::DMatrix<f64>>, MatioError>>()?,
            modes: Arc::new(vec![0.0; N_MODE]),
            actuators: vec![0.0; N_ACTUATOR],
        })
    }
}

impl Update for ModalToZonal {
    fn update(&mut self) {
        let _ = mem::replace(
            &mut self.actuators,
            self.modes
                .chunks(N_MODE)
                .zip(self.mats.iter())
                .map(|(modes, mat)| mat * na::DVector::from_column_slice(modes))
                .flat_map(|actuators| actuators.as_slice().to_vec())
                .collect(),
        );
    }
}

impl Read<M2modes> for ModalToZonal {
    fn read(&mut self, data: Data<M2modes>) {
        self.modes = data.into_arc();
    }
}

impl Write<M2ASMAsmCommand> for ModalToZonal {
    fn write(&mut self) -> Option<Data<M2ASMAsmCommand>> {
        Some(self.actuators.clone().into())
    }
}
