use std::fs::File;

use crseo::{gmt::GmtM1, Atmosphere, FromBuilder, RayTracing, Source};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{print::Print, Gain, Signals};
use gmt_dos_clients_crseo::{
    calibration::{algebra::Collapse, Calibration, CalibrationMode},
    sensors::DispersedFringeSensor,
    DeviceInitialize, DispersedFringeSensorProcessing, OpticalModel,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, SegmentPiston,
    },
};
use skyangle::Conversion;

const DFS_CAMERA_EXPOSURE: usize = 10;
const DFS_FFT_EXPOSURE: usize = 1000;

type DFS = DispersedFringeSensor<DFS_CAMERA_EXPOSURE, DFS_FFT_EXPOSURE>;
type DFS11 = DispersedFringeSensor<1, 1>;
type DFSP11 = DispersedFringeSensorProcessing;

/*
on-axis:  #    0: [[-1012, +2094, -3171, +4130, -5288, +6327]]
8' off-axis:  #    0: [[-1004, +2047, -3183, +4160, -5269, +6254]]
3 x 8' off-axis:  #    0: [[-1023, +2042, -3164, +4171, -5256, +6269]]
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sampling_frequency = 1000f64;
    let atm_builder = Atmosphere::builder().ray_tracing(
        RayTracing::default()
            .field_size(20f64.from_arcmin())
            .duration(30.)
            .filepath("dfs-atmosphere.bin"),
    );

    let src_builder = Source::builder()
        .band("J")
        .size(3)
        .on_ring(8f32.from_arcmin());

    let calib_m1_tz = if let Ok(file) = File::open("calib_m1_tz.pkl") {
        serde_pickle::from_reader(&file, Default::default()).unwrap()
    } else {
        let mut calib_m1_tz = <DFSP11 as Calibration<GmtM1>>::calibrate_serial(
            &OpticalModel::<DFS11>::builder()
                .source(src_builder.clone())
                .sensor(
                    DFS11::builder()
                        .source(src_builder.clone())
                        .nyquist_factor(3.),
                ),
            [CalibrationMode::RBM([None, None, Some(1e-6), None, None, None]); 6],
        )?
        .collapse();
        calib_m1_tz.pseudoinverse();
        println!("{calib_m1_tz}");
        let mut file = File::create("calib_m1_tz.pkl")?;
        serde_pickle::to_writer(&mut file, &calib_m1_tz, Default::default())?;
        calib_m1_tz
    };

    let om_builder = OpticalModel::<DFS>::builder()
        .sampling_frequency(sampling_frequency)
        .source(src_builder.clone())
        .atmosphere(atm_builder)
        .sensor(
            DFS::builder()
                .source(src_builder.clone())
                .nyquist_factor(3.),
        );
    let om = om_builder.clone().build()?;
    let n = om.sensor().as_ref().unwrap().frame_size();
    let n_fft = om.sensor().as_ref().unwrap().fft_size();
    dbg!((n, n_fft));

    let mut dfs_processor = DispersedFringeSensorProcessing::new();
    om_builder.initialize(&mut dfs_processor);

    {
        let print = Print::default();

        let sid = 5;
        let m1_rbm = (1..=6).fold(Signals::new(42, 1000), |s, sid| {
            s.channel(
                2 + (sid - 1) * 6,
                (-1f64).powi(sid as i32) * (sid as f64) * 1e-6,
            )
        });

        let to_nm = Gain::new(vec![1e9; 6 * 6]);

        actorscript!(
            1: m1_rbm[M1RigidBodyMotions] -> om
            1000: om[DfsFftFrame<Dev>]! -> dfs_processor[Intercepts]
                    -> calib_m1_tz[SegmentPiston] -> to_nm[SegmentPiston] -> print
        );
    }

    /*     let timer: Timer = Timer::new(1000);
    actorscript!(
        #[model(name = dfs_frames)]
        1: timer[Tick] -> om
        100: om[DfsFftFrame<Host>]!$
    ); */

    /*     // println!("DFS CAMERA FRAME");
    let mut log = logging_10.lock().await;

    let data: Vec<Vec<f32>> = log.iter("Host")?.collect();

    dbg!(data.len());
    dbg!(data[0].len());

    data.iter()
        .enumerate()
        .for_each(|(i, row)| println!("{}: {}", i, row.iter().sum::<f32>())); */

    // {
    //     println!("DFS FFT FRAME");
    //     let mut log = logging.lock().await;

    //     dbg!(data.len());
    //     data.iter()
    //         .enumerate()
    //         .for_each(|(i, row)| println!("{}: {}", i, row.iter().sum::<f32>()));
    // }
    // dbg!(data.iter().sum::<f32>());

    // let _: complot::Heatmap = ((data.last().unwrap().as_slice(), (n_fft, n_fft)), None).into();

    Ok(())
}

/*
 3514214500000000.0
35142468000000000
7028429000000000.0


1952521700000000000000000000000
1952521700000000000000000000000
*/
