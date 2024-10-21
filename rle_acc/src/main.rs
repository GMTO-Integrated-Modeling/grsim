//
// GRSIM script to simulate the effect of RLE ground accelerations
//

use gmt_dos_actors::actorscript;
use gmt_dos_clients::Source; //{Signals, Signal, } 
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::OSS00GroundAcc,
        outputs::{OSSPayloads6D,OSSHardpointD,OSSM1Lcl,Pier6D,OSS00Ground6D},//,OSSHardpointForce
    },
    gmt_m1::assembly,
    mount::{MountEncoders, MountTorques, }, // MountSetPoint,
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

    let sim_sampling_frequency = 1000;
    //let n_step = sssha_length;//10000; //
    let mut fem = FEM::from_env()?;    
    let m1_calibration = Calibration::new(&mut fem);



    for id in 4..=7 {

        dbg!(format!("RLE #{:02} dataset",id));
        // RLE data path
        let sssha_path = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap())
            .join("data")
            .join(format!("RLE{:02}_1kHz_dt.mat",id));
        let acc_mat: na::DMatrix<f64> = MatFile::load(&sssha_path)?.var(format!("RLE{:02}_1kHz_dt",id))?;
        let (_sssha_length, n_sssha_dim) = acc_mat.shape();
        dbg!(acc_mat.shape());

        // DISCRETE STATE-SPACE MODEL (from FEM)
        let sids: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7];
        let state_space =
            DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem.clone())
                .sampling(sim_sampling_frequency as f64)
                .proportional_damping(2. / 100.)
                // .max_eigen_frequency(95f64)
                .including_mount()
                .including_m1(Some(sids.clone()))?
                .ins::<OSS00GroundAcc>()
                .outs::<OSSM1Lcl>()
                .outs::<Pier6D>()
                .outs::<OSS00Ground6D>()
                .outs::<OSSPayloads6D>()
                .outs::<OSSHardpointD>()
                // .outs::<OSSHardpointForce>()       
                .use_static_gain_compensation()
                .build()?;
        println!("{state_space}");

        // FEM
        let fem = state_space;
        // MOUNT CONTROL
        let mount = Mount::new();
        // M1 control model
        let m1 = M1::<ACTUATOR_RATE>::new(&m1_calibration)?;

        // RLE-SSSHA DATA
        let sssha_source = Source::new(acc_mat.transpose().as_slice().to_vec(), n_sssha_dim);
    // let sssha_source = Signals::new(3, n_step)
    //     //.channel(0, Signal::Constant(5./100.))
    //     .channel(1, Signal::Sinusoid{
    //         amplitude: 25./100.,
    //         sampling_frequency_hz: 1000f64,
    //         frequency_hz: 4.0f64,
    //         phase_s: 0f64
    //     })
    //     .channel(2, Signal::Sinusoid{
    //         amplitude: 20./100.,
    //         sampling_frequency_hz: 1000f64,
    //         frequency_hz: 2.5f64,
    //         phase_s: 0.1f64
    //     });
        

        // DSL
        actorscript! {
        #[labels(fem = "Telescope\nStructure", mount = "Mount\nControl", sssha_source = "SSSHA Acc")]
        1: sssha_source[OSS00GroundAcc] -> fem
        // Mount control loop
        //1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount
        1: fem[MountEncoders]! -> mount[MountTorques] -> fem
        // M1 SA Force loop      
        1: fem[assembly::M1HardpointsMotion]! 
            -> {m1}[assembly::M1ActuatorAppliedForces] -> fem    
        // LOG
        1: fem[OSSPayloads6D]${162}
        1: sssha_source[OSS00GroundAcc]${3}
        1: fem[Pier6D]${12}
        1: fem[OSS00Ground6D]${6}
        1: fem[OSSHardpointD]${84}
        //1: fem[OSSHardpointForce]${42}
        1: fem[OSSM1Lcl]${42}
        1: fem[MountEncoders]${14}
        }
        logging_1.lock().await.to_parquet(format!("model-20240408_1535-RLE{:02}_wiM1c_wiSGMC_1.parquet",id))?;
    }
    Ok(())
}
