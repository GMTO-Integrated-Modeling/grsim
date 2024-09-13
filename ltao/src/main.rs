use std::fs::File;

use complot::{Config, Heatmap};
use crseo::{
    gmt::GmtM2,
    imaging::{ImagingBuilder, LensletArray},
    FromBuilder, Gmt, Imaging, Source,
};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{print::Print, Gain, Signal, Signals, Timer};
use gmt_dos_clients_crseo::{
    calibration::{Calibrate, CalibrationMode,Reconstructor},
    sensors::{NoSensor, WaveSensor, WaveSensorBuilder},
    OpticalModel,Centroids
};
use gmt_dos_clients_io::{
    gmt_m1::{segment::RBM, M1RigidBodyMotions},
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{Dev, Frame, SegmentWfeRms, SensorData, Wavefront, WfeRms},
};
use interface::{Tick, Write, UID};
use skyangle::Conversion;

const M2_N_MODE: usize = 66;

#[derive(UID)]
#[alias(name = Wavefront, client = OpticalModel<Imaging>, traits = Write, Size)]
pub enum M2Wavefront {}
#[derive(UID)]
#[alias(name = Wavefront, client = OpticalModel<Imaging>, traits = Write, Size)]
pub enum M1Wavefront {}
#[derive(UID)]
#[alias(name = Wavefront, client = OpticalModel<WaveSensor>, traits = Write, Size)]
pub enum OffAxisWavefront {}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Wavefront sensor (Shack-Hartmann)
    let imgr_builder: ImagingBuilder = Imaging::builder()
        .lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(16))
        .lenslet_flux(0.75);
    // Centroiding data processor
    let mut centroids = Centroids::try_from(&imgr_builder)?;

    // On-axis optical model
    let om_builder = OpticalModel::<Imaging>::builder()
        .gmt(Gmt::builder().m2("Karhunen-Loeve", M2_N_MODE))
        .sensor(imgr_builder);

    // Calibration of M2 Karhunen-Loeve modes
    let mut calib_m2_modes = <Centroids as Calibrate<GmtM2>>::calibrate(
        om_builder.clone(),
        CalibrationMode::modes(M2_N_MODE, 1e-6).start_from(2),
    )?;
    calib_m2_modes.pseudoinverse();
    println!("{calib_m2_modes}");

    let mut om = om_builder.build()?;
    centroids.setup(&mut om);
    dbg!(centroids.n_valid_lenslets());

    // M1 Rx & Ry commands
    let mut m1_rxy = [[0.; 2]; 7];
    m1_rxy[0][0] = 1f64 / 10.; // arcsec
    m1_rxy[1][0] = -1f64 / 20.; // arcsec
    m1_rxy[2][1] = -1f64 / 10.; // arcsec
    m1_rxy[5][1] = -1f64 / 5.; // arcsec
    m1_rxy[6][0] = 1f64 / 10.; // arcsec
    let n_step = 1;
    let m1_rbm = m1_rxy
        .iter()
        .enumerate()
        .fold(Signals::new(42, n_step), |s, (i, rxy)| {
            s.channel(i * 6 + 3, Signal::Constant(rxy[0].from_arcsec()))
                .channel(i * 6 + 4, Signal::Constant(rxy[1].from_arcsec()))
        });

    let print = Print::default();

    // Off-axis optical model
    let srcs = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let oom = OpticalModel::<WaveSensor>::builder()
        .gmt(Gmt::builder().m2("Karhunen-Loeve", M2_N_MODE))
        .source(srcs.clone())
        .sensor(WaveSensorBuilder(
            OpticalModel::<NoSensor>::builder().source(srcs.clone()),
        ))
        .build()?;

    // Active optics reconstructor
    let m1_to_agws: Reconstructor = serde_pickle::from_reader(
        File::open("src/bin/linear_model/m1_to_agws.pkl")?,
        Default::default(),
    )?;
    println!("{m1_to_agws}");

    // WAVEFRONT SENSING
    actorscript!(
        #[model(name = wavefront_sensing)]
        #[labels(calib_m2_modes = "Karhunen-Loeve\nreconstructor",
        m1_rbm = "M1 Rx & Ry RBMS",
        om = "Optical Model:\nOn-axis SH-WFS",
        oom = "Optical Model:\nAGWS")]
        // commands
        //  * on-axis
        1: m1_rbm[M1RigidBodyMotions] -> om
        //  * off-axis
        1: m1_rbm[M1RigidBodyMotions] -> oom
        // on-axis wavefront sensing
        1: om[Frame<Dev>] -> centroids[SensorData] -> calib_m2_modes
        // stats
        1: om[SegmentWfeRms<-9>] -> print
        1: om[WfeRms<-9>] -> print
    );

    // RECONSTRUCTION & CORRECTION
    let timer: Timer = Timer::new(1);
    let neg = Gain::new(vec![-1.; M2_N_MODE * 7]);
    actorscript!(
        #[model(name = wavefront_correction)]
        #[labels(calib_m2_modes = "Karhunen-Loeve\nreconstructor",
        neg = ".* -1",
        m1_to_agws = "M1 Rx & Ry\nreconstructor",
        om = "Optical Model:\nOn-axis SH-WFS",
        oom = "Optical Model:\nAGWS")]
        1: timer[Tick] -> om
        // on-axis reconstruction & correction
        1: calib_m2_modes[M2ASMAsmCommand] -> neg[M2ASMAsmCommand]-> om[Wavefront]$
        // off-axis wavefront sensing
        1: neg[M2ASMAsmCommand]-> oom[OffAxisWavefront]$ -> m1_to_agws
        // stats
        1: om[SegmentWfeRms<-9>] -> print
        1: om[WfeRms<-9>] -> print
    );

    print!("      ");
    ["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]
        .into_iter()
        .for_each(|r| print!("{:7} ", r));
    println!("");
    <Reconstructor as Write<M1RigidBodyMotions>>::write(&mut *m1_to_agws.lock().await)
        .unwrap()
        .chunks(6)
        .zip(&m1_rxy)
        .enumerate()
        .for_each(|(i, (data, rxy))| {
            println!(
                " #{}: {:+.3?} {:+.3?}",
                i + 1,
                data.iter()
                    .map(|x| x.to_arcsec())
                    .collect::<Vec<_>>(),
                rxy
            )
        });

    {
        let mut oml = om.lock().await;
        let n = oml.src.pupil_sampling() as usize;
        let phase: Vec<_> = oml.src.phase().iter().map(|x| *x * 1e9).collect();

        let _: Heatmap = (
            (phase.as_slice(), (n, n)),
            Some(Config::new().filename("on-axis_m2-corrected_wavefront.png")),
        )
            .into();
    }

    {
        let mut ooml = oom.lock().await;
        let n = ooml.src.pupil_sampling() as usize;
        let phase: Vec<_> = ooml
            .sensor()
            .unwrap()
            .phase()
            .iter()
            .map(|x| *x * 1e9)
            .collect();

        phase.chunks(n * n).enumerate().for_each(|(i, phase)| {
            let _: Heatmap = (
                (phase, (n, n)),
                Some(Config::new().filename(format!("off-axis_m2-corrected_wavefront_{i}.png"))),
            )
                .into();
        });
    }

    Ok(())
}
