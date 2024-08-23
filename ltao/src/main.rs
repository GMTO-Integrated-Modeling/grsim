use crseo::gmt::GmtM2;
use crseo::imaging::{ImagingBuilder, LensletArray};
use crseo::{FromBuilder, Gmt, Imaging, Source};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::print::Print;
use gmt_dos_clients::{Gain, Signal, Signals, Timer};
use gmt_dos_clients_crseo::{
    Calibrate, CalibrationMode, Centroids, NoSensor, OpticalModel, OpticalModelBuilder, Wave,
    WavefrontBuilder, WavefrontSensor,
};
use gmt_dos_clients_io::gmt_m1::segment::RBM;
use gmt_dos_clients_io::gmt_m2::asm::M2ASMAsmCommand;
use gmt_dos_clients_io::optics::{Dev, Frame, SegmentWfeRms, SensorData, Wavefront, WfeRms};
use interface::{Tick, UID};
use skyangle::Conversion;

const M2_N_MODE: usize = 66;

#[derive(UID)]
#[alias(name = Wavefront, client = OpticalModel<Imaging>, traits = Write, Size)]
pub enum M2Wavefront {}
#[derive(UID)]
#[alias(name = Wavefront, client = OpticalModel<Imaging>, traits = Write, Size)]
pub enum M1Wavefront {}
#[derive(UID)]
#[alias(name = Wavefront, client = OpticalModel<Wave>, traits = Write, Size)]
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
    let n_step = 1;
    let m1_s1_rbm =
        Signals::new(6, n_step).channel(3, Signal::Constant(dbg!(1f64 / 10.).from_arcsec()));
    let m1_s7_rbm =
        Signals::new(6, n_step).channel(3, Signal::Constant((1f64 / 10.).from_arcsec()));
    let m1_s3_rbm =
        Signals::new(6, n_step).channel(4, Signal::Constant((-1f64 / 10.).from_arcsec()));

    let print = Print::default();

    // Off-axis optical model
    let srcs = Source::builder().size(3).on_ring(6f32.from_arcmin());
    let oom = OpticalModel::<Wave>::builder()
        .gmt(Gmt::builder().m2("Karhunen-Loeve", M2_N_MODE))
        .source(srcs.clone())
        .sensor(WavefrontBuilder(
            OpticalModel::<NoSensor>::builder().source(srcs),
        ))
        .build()?;

    // WAVEFRONT SENSING
    actorscript!(
        #[model(name = wavefront_sensing)]
        // commands
        //  * on-axis
        1: m1_s1_rbm[RBM<1>] -> om
        1: m1_s3_rbm[RBM<3>] -> om
        1: m1_s7_rbm[RBM<7>] -> om
        //  * off-axis
        1: m1_s1_rbm[RBM<1>] -> oom
        1: m1_s3_rbm[RBM<3>] -> oom
        1: m1_s7_rbm[RBM<7>] -> oom
        // wavefront sensing
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
        1: timer[Tick] -> om
        // on-axis reconstruction & correction
        1: calib_m2_modes[M2ASMAsmCommand] -> neg[M2ASMAsmCommand]-> om[Wavefront]$
        // off-axis wavefront
        1: neg[M2ASMAsmCommand]-> oom[WavefrontSensor]$
        // stats
        1: om[SegmentWfeRms<-9>] -> print
        1: om[WfeRms<-9>] -> print

    );
    Ok(())
}
