use std::{
    env,
    fmt::Debug,
    fs::File,
    marker::PhantomData,
    ops::{Add, AddAssign, Deref, DerefMut, Sub, SubAssign},
    path::Path,
    sync::Arc,
    time::Instant,
};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    integrator::Offset,
    leftright::{self, Left as Piston, Right as NoPiston},
    once::Once,
    Integrator, Tick, Timer,
};
use gmt_dos_clients_crseo::{
    crseo::{
        atmosphere,
        wavefrontsensor::{
            Calibration, GmtSegmentation, LensletArray, PhaseSensor, PistonSensor, Pyramid,
            SegmentCalibration, Stroke, TruncatedPseudoInverse,
        },
        Atmosphere, Builder, FromBuilder, Gmt, SegmentWiseSensorBuilder, WavefrontSensorBuilder,
    },
    GuideStar, OpticalModel, ResidualM2modes, ResidualPistonMode, WavefrontSensor,
};
use gmt_dos_clients_io::optics::{M2modes, SegmentPiston, WfeRms};
use interface::{select::Select, units::NM, Data, Read, UniqueIdentifier, Update};
use nalgebra::{DefaultAllocator, Dim, Dyn};
use nanorand::{Rng, WyRand};

use grsim_ngao::Processing;

// const PYWFS_READOUT: usize = 8;
// const PYWFS: usize = 1;
// const HDFS: usize = 10;

#[derive(Debug, Default)]
pub struct Print<T> {
    counter: usize,
    data: Option<Arc<Vec<T>>>,
}

impl<T> Update for Print<T>
where
    T: Send + Sync + Debug,
{
    fn update(&mut self) {
        if let Some(data) = self.data.as_ref() {
            println!(" #{:>5}: {:4.2?}", self.counter, data);
            self.counter += 1;
        }
    }
}

impl<T, U> Read<U> for Print<T>
where
    T: Send + Sync + std::fmt::Debug,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        self.data = Some(data.as_arc());
    }
}









#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    dbg!(&data_repo);
    env::set_var("DATA_REPO", &data_repo);
    env::set_var(
        "GMT_MODES_PATH",
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../data"),
    );
    env::set_var("FLOWCHART", "dot");

    let sampling_frequency = 1_000usize; // Hz
    let sim_duration = 1usize;
    let n_sample = sim_duration * sampling_frequency;

    let n_lenslet = 92;
    let n_mode: usize = 450; //env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());

    // Wavefront phase sensor
    // let builder = PhaseSensor::builder()
    //     .lenslet(n_lenslet, 4)
    //     .wrapping(760e-9 * 0.5);
    let mut builder = Pyramid::builder()
        .lenslet_array(LensletArray {
            n_side_lenslet: n_lenslet,
            n_px_lenslet: 10,
            d: 0f64,
        })
        .modulation(2., 64);
    let src_builder = builder.guide_stars(None);

    let m2_modes = "ASM_DDKLs_S7OC04184_675kls";
    let stroke0 = 25e-9;

    // M2 Karhunen-Loeve segment calibration
    // let mut slopes_mat: Calibration =
    let now = Instant::now();
    let mut slopes_mat: Calibration = /*builder.clone().calibrate(
        SegmentCalibration::modes(m2_modes, 1..n_mode, "M2", Stroke::RadialOrder(stroke0)),
        src_builder.clone(),
    );
    eprintln!(
        "M2 {}modes/segment calibrated in {}s",
        n_mode,
        now.elapsed().as_secs()
    );
    serde_pickle::to_writer(
        &mut File::create(format!("slopes_mat-{n_mode}.pkl"))?,
        &slopes_mat,
        Default::default(),
    )?; */
    serde_pickle::from_reader(
        File::open(format!("slopes_mat-{n_mode}_no-truss.pkl"))?,
        Default::default(),
    )?;
    let mut truncation = vec![None; 7];
    truncation[6] = Some(TruncatedPseudoInverse::EigenValues(18));
    slopes_mat.pseudo_inverse(None).unwrap();

    builder.piston_sensor(&slopes_mat, GmtSegmentation::Outers)?;

    // Segment piston calibration
    let piston_builder = PistonSensor::builder().pupil_sampling(builder.pupil_sampling());
    let piston_mat = {
        let now = Instant::now();
        let mut piston_mat = piston_builder.calibrate(
            SegmentCalibration::modes(m2_modes, 0..1, "M2", stroke0),
            src_builder.clone(),
        );
        eprintln!(
            "M2 {}modes/segment calibrated in {}s",
            1,
            now.elapsed().as_secs()
        );
        piston_mat.pseudo_inverse(None).unwrap();
        let p2m = piston_mat.concat_pinv();
        dbg!(&p2m);
        piston_mat
    };

    // Atmospher builder
    let atm_builder = Atmosphere::builder().ray_tracing(
        atmosphere::RayTracing::default()
            .duration(5f64)
            .filepath(&data_repo.join("atmosphere.bin")),
    );

    // GMT optical model
    let mut piston_dist = vec![0f32; 7];
    let mut rng = WyRand::new();
    piston_dist.iter_mut().take(7).for_each(|x| {
        *x = (2. * rng.generate::<f32>() - 1.) * 500e-9;
    });
    println!(
        "Piston dist: {:4.0?}",
        piston_dist.iter().map(|x| x * 1e9).collect::<Vec<_>>()
    );
    let gom = OpticalModel::builder()
        .gmt(Gmt::builder().m2(m2_modes, n_mode))
        .source(src_builder.clone())
        .atmosphere(atm_builder)
        // .piston(piston_dist)
        .sampling_frequency(sampling_frequency as f64)
        .build()?;
    let sensor = WavefrontSensor::<_, 1>::new(builder.build()?, slopes_mat);
    let piston_sensor = WavefrontSensor::<_, 100>::new(piston_builder.build()?, piston_mat);

    let timer: Timer = Timer::new(10); //n_sample);

    let no_piston_ctrl = Integrator::<NoPiston<ResidualM2modes>>::new((n_mode - 1) * 7).gain(0.5);
    let pym_ctrl = Integrator::<ResidualM2modes>::new(n_mode * 7).gain(0.5);
    let piston_ctrl = Integrator::<Piston<ResidualM2modes>>::new(7).gain(0.2);
    // let piston_ctrl = Integrator::<ResidualPistonMode>::new(7).gain(0.1);

    let (split_piston, merge_piston) =
        leftright::split_merge_chunks_at::<ResidualM2modes, M2modes>(n_mode, 1);
    let offset_piston = Once::<ResidualPistonMode>::new();

    type PistonOffset = Offset<ResidualPistonMode>;
    type WfeRmsNm = NM<WfeRms>;

    let act_print = Print::<f64>::default();

    actorscript! {
    #[model(state = completed)]
    1: timer[Tick]
        ->  &gom[WfeRmsNm] -> act_print };

    /*         actorscript! {
        #[model(state = completed)]
        1: timer[Tick]
            -> &gom[GuideStar]
                -> sensor("PWFS")[ResidualM2modes]
                    -> split_piston("Split Piston\nfrom other modes")[Piston<ResidualM2modes>]
                        -> piston_ctrl("Piston\nControl")[Piston<ResidualM2modes>]!
                            -> merge_piston("Merge Piston\nwith other modes")
        1: split_piston("Split Piston\nfrom other modes")[NoPiston<ResidualM2modes>]
            -> no_piston_ctrl("No Piston Modes\nControl")[NoPiston<ResidualM2modes>]!
                -> merge_piston("Merge Piston\nwith other modes")[M2modes] -> &gom
        1: &gom[GuideStar] -> piston_sensor("HDFS")
        10: piston_sensor("HDFS")[ResidualPistonMode] -> offset_piston
        1: offset_piston[PistonOffset] -> piston_ctrl("Piston\nControl")
        // 1: &gom[WfeRmsNm]~
        1: &gom[NM<Select<SegmentPiston, 0>>]~
        1: &gom[NM<Select<SegmentPiston, 1>>]~
        1: &gom[NM<Select<SegmentPiston, 2>>]~
        1: &gom[NM<Select<SegmentPiston, 6>>]~
        // 1: &gom[NM<Select<SegmentPiston, 4>>]~
        // 1: &gom[NM<Select<SegmentPiston, 2>>]~
        // 1: &gom[NM<Select<SegmentPiston, 1>>]~
    } */

    /*
    // Truth piston sensor only
         actorscript! {
        #[model(state = completed)]
        1: timer[Tick]
            -> &gom[GuideStar]
                -> piston_sensor("HDFS")
        100: piston_sensor("HDFS")[ResidualPistonMode]  -> piston_ctrl("Piston\nControl")
        1: piston_ctrl("Piston\nControl")[M2modes]! -> &gom
        // 1: &gom[WfeRmsNm]~
        1: &gom[NM<Select<SegmentPiston, 0>>]~
        1: &gom[NM<Select<SegmentPiston, 1>>]~
        1: &gom[NM<Select<SegmentPiston, 2>>]~
        1: &gom[NM<Select<SegmentPiston, 6>>]~
        // 1: &gom[NM<Select<SegmentPiston, 4>>]~
        // 1: &gom[NM<Select<SegmentPiston, 2>>]~
        // 1: &gom[NM<Select<SegmentPiston, 1>>]~
    } */

    /*     // Pyramid high-order sensor only
    actorscript! {
        #[model(state = completed)]
        1: timer[Tick]
            -> &gom[GuideStar]
                -> sensor("PWFS")[ResidualM2modes]
                        -> pym_ctrl("Pyramid\nControl")[ResidualM2modes]! -> &gom
        1: &gom[WfeRmsNm]~
        1: &gom[NM<Select<SegmentPiston, 0>>]~
        1: &gom[NM<Select<SegmentPiston, 1>>]~
        1: &gom[NM<Select<SegmentPiston, 2>>]~
        1: &gom[NM<Select<SegmentPiston, 6>>]~
        // 1: &gom[NM<Select<SegmentPiston, 4>>]~
        // 1: &gom[NM<Select<SegmentPiston, 2>>]~
        // 1: &gom[NM<Select<SegmentPiston, 1>>]~
    } */

    /*
    // Pyramid piston sensor only
         actorscript! {
        #[model(state = completed)]
        1: timer[Tick]
            -> &gom[GuideStar]
                -> sensor("PWFS")[ResidualM2modes]
                    -> split_piston("Split Piston\nfrom other modes")[Piston<ResidualM2modes>]
                        -> piston_ctrl("Piston\nControl")[M2modes]! -> &gom
        // 1: &gom[WfeRmsNm]~
        1: &gom[NM<Select<SegmentPiston, 0>>]~
        1: &gom[NM<Select<SegmentPiston, 1>>]~
        1: &gom[NM<Select<SegmentPiston, 2>>]~
        1: &gom[NM<Select<SegmentPiston, 6>>]~
        // 1: &gom[NM<Select<SegmentPiston, 4>>]~
        // 1: &gom[NM<Select<SegmentPiston, 2>>]~
        // 1: &gom[NM<Select<SegmentPiston, 1>>]~
    } */

    Ok(())
}
