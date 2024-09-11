//
//
//

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Signals, Source};
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::OSS00GroundAcc,
        outputs::{OSSPayloads6D,OSSHardpointD},
    },
    gmt_m1::assembly, //{, M1RigidBodyMotions},    
    mount::{MountEncoders, MountSetPoint, MountTorques},
};

use gmt_dos_clients_m1_ctrl::{Calibration, M1};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;

use matio_rs::MatFile;
use nalgebra as na;

use std::{env, path::Path};

const ACTUATOR_RATE: usize = 10;

/*
. setup.sh
cargo run --release --bin rle_acc
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env::set_var("DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("data"),            
    );

    // RLE data path
    let sssha_path = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap())
        .join("data")
        .join("RLE06_1kHz_dt.mat");
    let acc_mat: na::DMatrix<f64> = MatFile::load(&sssha_path)?.var("RLE06_1kHz_dt")?;
    let (sssha_length, n_sssha_dim) = acc_mat.shape();
    dbg!(acc_mat.shape());


    let sim_sampling_frequency = 1000;
    let n_step = sssha_length;//1000; //
    let mut fem = Option::<FEM>::None;    
    let m1_calibration = Calibration::new(fem.get_or_insert(FEM::from_env()?));

    // FEM MODEL
    let sids: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7];
    let state_space =
        DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem.unwrap_or(FEM::from_env()?))
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            //.max_eigen_frequency(75f64)
            .including_mount()
            .including_m1(Some(sids.clone()))?
            .ins::<OSS00GroundAcc>()                        
            .outs::<OSSPayloads6D>()            
            //.use_static_gain_compensation()
            .build()?;
    println!("{state_space}");

    // FEM
    let fem = state_space;
    // M1 control model
    let m1 = M1::<ACTUATOR_RATE>::new(&m1_calibration)?;

    // MOUNT SET-POINT
    let setpoint = Signals::new(3, n_step); //.channel(1, Signal::Constant(1f64.from_arcsec()));    
    // MOUNT CONTROL
    let mount = Mount::new();
    // RLE-SSSHA DATA
    let sssha_source = Source::new(acc_mat.transpose().as_slice().to_vec(), n_sssha_dim);
    // M1 loop reference signals
    let actuators = Signals::new(6 * 335 + 306, n_step);
    let m1_rbm = Signals::new(6 * 7, n_step);
    
    // DSL
    actorscript! {
    #[labels(fem = "GMT FEM", mount = "Mount\nControl", sssha_source = "SSSHA Acc")]
    1: setpoint[MountSetPoint] -> mount[MountTorques] 
        -> fem[MountEncoders]! -> mount    
        1: m1_rbm[assembly::M1RigidBodyMotions]
    -> {m1}[assembly::M1HardpointsForces]
        -> fem[assembly::M1HardpointsMotion]! -> {m1}
    1: actuators[assembly::M1ActuatorCommandForces]
            -> {m1}[assembly::M1ActuatorAppliedForces] -> fem
    1: sssha_source[OSS00GroundAcc] -> fem
    // LOG
    1: fem[OSSPayloads6D]${162}
    1: sssha_source[OSS00GroundAcc]${3}
    1: fem[OSSHardpointD]${84}

    }

    Ok(())
}
