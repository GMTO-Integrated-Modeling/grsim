//! M1 Rxy estimation with both LTWS & OIWFS

use std::{
    fs::{DirBuilder, File},
    path::Path,
};

use crseo::{
    gmt::GmtM2,
    imaging::{Detector, LensletArray},
    FromBuilder, Gmt, Source,
};
use gmt_dos_clients::gif::Frame as Png;
use gmt_dos_clients_crseo::{
    calibration::{
        algebra::{Block, Collapse, Expand},
        estimation::{closed_loop::ClosedLoopEstimation, Estimation},
        Calibration, CalibrationMode, ClosedLoopCalibration, GlobalCalibration, MirrorMode,
        Reconstructor,
    },
    centroiding::{CentroidKind, CentroidsProcessing, Full, ZeroMean},
    sensors::{
        builders::WaveSensorBuilder, Camera, DispersedFringeSensor, SegmentGradientSensor,
        WaveSensor,
    },
    DeviceInitialize, DispersedFringeSensorProcessing, OpticalModel, OpticalModelBuilder,
};
use gmt_dos_clients_io::optics::M2GlobalTipTilt;
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev, Frame, Host, SegmentTipTilt, SensorData, Wavefront, WfeRms,
    },
    Estimate,
};
use interface::{Read, Update, Write};
use ltao::M2_N_MODE;
use skyangle::Conversion;

const M1_N_MODE: usize = 27;
const AGWS_N_GS: usize = 3;

type DFS = DispersedFringeSensor<1, 1>;
type LTWS = Full;

fn main() -> anyhow::Result<()> {
    // GMT definition
    let gmt_builder = Gmt::builder()
        .m1("bending modes", M1_N_MODE)
        .m2("Karhunen-Loeve", M2_N_MODE)
        .m1_truss_projection(false);

    // LTWS: wavefront sensor
    let ltws = Camera::builder()
        .lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(32))
        // .detector(Detector::default().n_px_framelet(10))
        .lenslet_flux(0.75);
    // LTWS: centroids processing
    let ltws_centroids: CentroidsProcessing<LTWS> = CentroidsProcessing::<LTWS>::try_from(&ltws)?;
    // LTWS: optical model
    let ltws_om_builder = OpticalModel::<Camera<1>>::builder()
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("V"))
        .sensor(ltws);
    // LTWS: ASM Karhunen-Loeve calibration
    if !<LTWS as CentroidKind>::is_full() {
        panic!("LTWS is not Full");
    }
    let calib_file_name = format!("calib_ltws-{}_m2_modes.pkl", ltws_centroids.kind());
    let mut calib_m2_modes: Reconstructor = if let Ok(file) = File::open(&calib_file_name) {
        println!("loading {calib_file_name}");
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let mut calib_m2_modes = <CentroidsProcessing<LTWS> as Calibration<GmtM2>>::calibrate(
            &((&ltws_om_builder).into()),
            CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(2),
        )?;
        println!("{} cross-talks", calib_m2_modes.n_cross_talks());
        calib_m2_modes.pseudoinverse();
        serde_pickle::to_writer(
            &mut File::create(&calib_file_name)?,
            &calib_m2_modes,
            Default::default(),
        )?;
        calib_m2_modes
    };
    /* let mut calib_m2_modes: Reconstructor<MirrorMode> =
    if let Ok(file) = File::open(&calib_file_name) {
        println!("loading {calib_file_name}");
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let mut calib_m2_modes = <CentroidsProcessing<LTWS> as Calibration<GmtM2>>::calibrate(
            &((&ltws_om_builder).into()),
            CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(2),
        )?;
        println!("{} cross-talks", calib_m2_modes.n_cross_talks());
        // if !<LTWS as CentroidKind>::is_full() {
        let mut recon = calib_m2_modes.collapse();
        recon.pseudoinverse();
        serde_pickle::to_writer(
            &mut File::create(&calib_file_name)?,
            &recon,
            Default::default(),
        )?;
        recon
    }; */
    println!("{calib_m2_modes}");

    // M1 S1 Rx
    let sid = 1;
    let r_xy = "Rx";
    let dir = format!("M1S{sid}{r_xy}");
    let results_path = Path::new(&dir);
    println!("Results path: {:?}", results_path);
    DirBuilder::new().recursive(true).create(results_path)?;
    let mut cmd = vec![0f64; 42];
    cmd[match r_xy {
        "Rx" => 3,
        "Ry" => 4,
        _ => panic!("expected Rx or Ry, found {r_xy}"),
    } + (sid - 1) * 6] = 250f64.from_mas();
    // cmd[2] = 250e-9;

    /* let results_path = Path::new("M1GTTx");
    println!("Results path: {:?}", results_path);
    DirBuilder::new().recursive(true).create(results_path)?;
    let cmd =
        geotrans::Mirror::<geotrans::M1>::tiptilt_2_rigidbodymotions((50f64.from_mas(), 0f64)); */

    // M2 modes
    let m2_bm_e = <CentroidsProcessing as Estimation<M1RigidBodyMotions>>::estimate(
        &ltws_om_builder,
        &mut calib_m2_modes,
        &cmd,
    )?;
    println!("M2 modes for M1 S1 Rx");
    m2_bm_e.chunks(M2_N_MODE).enumerate().for_each(|(i, x)| {
        println!(
            "S{}: {:+6.0?}",
            i + 1,
            x[..6].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
        )
    });
    let m2_bm_e_zm = <CentroidsProcessing<ZeroMean> as Estimation<M1RigidBodyMotions>>::estimate(
        &ltws_om_builder,
        &mut calib_m2_modes,
        &cmd,
    )?;
    println!("M2 modes for M1 S1 Rx (w/ ZeroMean)");
    m2_bm_e_zm.chunks(M2_N_MODE).enumerate().for_each(|(i, x)| {
        println!(
            "S{}: {:+6.0?}",
            i + 1,
            x[..6].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
        )
    });
    // OIWFS TT7
    let seggrad_om = OpticalModel::<SegmentGradientSensor>::builder().gmt(gmt_builder.clone());

    let mut calib_oiwfs_tt7: Reconstructor = if let Ok(file) = File::open("calib_oiwfs_tt7.pkl") {
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let mut calib_oiwfs_tt7 = <SegmentGradientSensor as Calibration<GmtM2>>::calibrate(
            &seggrad_om,
            CalibrationMode::modes(M2_N_MODE, 1e-7)
                .start_from(2)
                .ends_at(3),
        )?;
        calib_oiwfs_tt7.pseudoinverse();
        serde_pickle::to_writer(
            &mut File::create("calib_oiwfs_tt7.pkl")?,
            &calib_oiwfs_tt7,
            Default::default(),
        )?;
        calib_oiwfs_tt7
    };
    println!("{calib_oiwfs_tt7}");

    let m2_bm_e_tt7 = <SegmentGradientSensor as Estimation<M2ASMAsmCommand>>::estimate(
        &seggrad_om,
        &mut calib_oiwfs_tt7,
        m2_bm_e_zm.as_slice(),
    )?;
    println!("M2 modes from OIWFS TT7");
    m2_bm_e_tt7
        .chunks(M2_N_MODE)
        .enumerate()
        .for_each(|(i, x)| {
            println!(
                "S{}: {:+6.0?}",
                i + 1,
                x[..6].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
            )
        });

    // OIWFS: imager
    let oiwfs_n_px = 255;
    let oiwfs = Camera::builder().detector(Detector::default().n_px_imagelet(oiwfs_n_px));
    // OIWFS: centroids processing
    let mut oiwfs_centroids: CentroidsProcessing = CentroidsProcessing::try_from(&oiwfs)?;
    // OIWFS: optical model
    let oiwfs_tt_om_builder = OpticalModel::<Camera<1>>::builder()
        .gmt(gmt_builder.clone())
        .source(Source::builder().band("K"))
        .sensor(oiwfs);
    // OIWFS: global tip-tilt calibration
    let mut calib_oiwfs: Reconstructor = if let Ok(file) = File::open("calib_oiwfs.pkl") {
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let mut calib_oiwfs = <CentroidsProcessing as GlobalCalibration<GmtM2>>::calibrate(
            &(&oiwfs_tt_om_builder).into(),
            gmt_dos_clients_crseo::calibration::CalibrationMode::GlobalTipTilt(100f64.from_mas()),
        )?;
        calib_oiwfs.pseudoinverse();
        serde_pickle::to_writer(
            &mut File::create("calib_oiwfs.pkl")?,
            &calib_oiwfs,
            Default::default(),
        )?;
        calib_oiwfs
    };
    println!("{calib_oiwfs}");

    let m2_tt = <CentroidsProcessing as ClosedLoopEstimation<
        Camera,
        M1RigidBodyMotions,
        CentroidsProcessing<ZeroMean>,
    >>::estimate_with_closed_loop_reconstructor(
        &oiwfs_tt_om_builder,
        &ltws_om_builder,
        &mut calib_oiwfs,
        &cmd,
        &mut calib_m2_modes,
    )?;
    println!(
        "M2 TT: {:?}",
        m2_tt.iter().map(|x| x.to_mas()).collect::<Vec<_>>()
    );

    oiwfs_tt_om_builder.initialize(&mut oiwfs_centroids);
    dbg!(oiwfs_centroids.n_valid_lenslets());

    let mut oiwfs_tt_om = oiwfs_tt_om_builder.build()?;
    println!("{oiwfs_tt_om}");

    // oiwfs_tt_zm_om.update();
    // <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut oiwfs_tt_om)
    //     .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut oiwfs_centroids, data));

    // oiwfs_centroids.update();
    // <CentroidsProcessing as Write<SensorData>>::write(&mut oiwfs_centroids)
    // <CentroidsProcessing as Write<SensorData>>::write(&mut oiwfs_centroids)
    //     .map(|data| println!("s_xy: {:?}mas", data.iter().map(|x| x).collect::<Vec<_>>()));

    // OIWFS: M1 segment tilt
    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut oiwfs_tt_om, cmd.clone().into());
    oiwfs_tt_om.update();
    let mut frame =
        Png::<f32, _>::new(results_path.join("oiwfs_m1.png"), oiwfs_n_px).filter(|x| x.cbrt());
    <OpticalModel<_> as Write<Frame<Host>>>::write(&mut oiwfs_tt_om)
        .map(|data| <Png<_, _> as Read<Frame<Host>>>::read(&mut frame, data));
    frame.update();
    frame.save()?;

    // OIWFS: M2 modes
    <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(
        &mut oiwfs_tt_om,
        m2_bm_e_zm.iter().map(|x| -*x).collect::<Vec<_>>().into(),
    );
    oiwfs_tt_om.update();

    let wfe_rms = <OpticalModel<_> as Write<WfeRms<-9>>>::write(&mut oiwfs_tt_om)
        .unwrap()
        .into_arc();
    dbg!(wfe_rms);
    let segment_tiptilt = <OpticalModel<_> as Write<SegmentTipTilt>>::write(&mut oiwfs_tt_om)
        .unwrap()
        .into_arc();
    println!("Segment Tip-Tilt");
    segment_tiptilt
        .iter()
        .take(7)
        .map(|x| x.to_mas())
        .zip(segment_tiptilt.iter().skip(7).map(|x| x.to_mas()))
        .enumerate()
        .for_each(|(i, (x, y))| println!("S{}: [{:+4.1},{:+4.1}]mas", i + 1, x, y));

    // OIWFS: processing
    <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut oiwfs_tt_om)
        .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut oiwfs_centroids, data));
    oiwfs_centroids.update();
    // OIWFS: estimation
    let oiwfs_data = <CentroidsProcessing as Write<SensorData>>::write(&mut oiwfs_centroids)
        .map(|data| {
            println!("s_xy: {:.2?}px", data.iter().map(|x| x).collect::<Vec<_>>());
            <Reconstructor as Read<SensorData>>::read(&mut calib_oiwfs, data.clone());
            data
        })
        .unwrap()
        .into_arc();
    let xy = (oiwfs_data[0].round() as i32, oiwfs_data[1].round() as i32);

    let mut frame = Png::<f32, _>::new(results_path.join("oiwfs_m1_m2_zm.png"), oiwfs_n_px)
        .filter(|x| x.cbrt())
        .cross(xy);
    <OpticalModel<_> as Write<Frame<Host>>>::write(&mut oiwfs_tt_om)
        .map(|data| <Png<_, _> as Read<Frame<Host>>>::read(&mut frame, data));
    frame.update();
    frame.save()?;

    let mut opd = Png::<f64>::new(results_path.join("oiwfs_opd_m1_m2_zm.png"), 512);
    <OpticalModel<_> as Write<Wavefront>>::write(&mut oiwfs_tt_om)
        .map(|data| <Png<_> as Read<Wavefront>>::read(&mut opd, data));
    opd.update();
    opd.save()?;

    calib_oiwfs.update();
    let estimate = <Reconstructor as Write<Estimate>>::write(&mut calib_oiwfs)
        .unwrap()
        .into_arc();
    println!(
        "Estimate: {:?}",
        estimate.iter().map(|x| x.to_mas()).collect::<Vec<_>>()
    );

    // OIWFS: correction
    <OpticalModel<_> as Read<M2GlobalTipTilt>>::read(
        &mut oiwfs_tt_om,
        vec![-estimate[0], -estimate[1]].into(),
    );
    oiwfs_tt_om.update();

    let wfe_rms = <OpticalModel<_> as Write<WfeRms<-9>>>::write(&mut oiwfs_tt_om)
        .unwrap()
        .into_arc();
    dbg!(wfe_rms);
    println!("Segment Tip-Tilt");
    let segment_tiptilt = <OpticalModel<_> as Write<SegmentTipTilt>>::write(&mut oiwfs_tt_om)
        .unwrap()
        .into_arc();
    segment_tiptilt
        .iter()
        .take(7)
        .map(|x| x.to_mas())
        .zip(segment_tiptilt.iter().skip(7).map(|x| x.to_mas()))
        .enumerate()
        .for_each(|(i, (x, y))| println!("S{}: [{:>+4.1},{:>+4.1}]mas", i + 1, x, y));

    let mut frame = Png::<f32, _>::new(results_path.join("oiwfs_correction.png"), oiwfs_n_px)
        .filter(|x| x.cbrt())
        .cross(xy);
    <OpticalModel<_> as Write<Frame<Host>>>::write(&mut oiwfs_tt_om)
        .map(|data| <Png<_, _> as Read<Frame<Host>>>::read(&mut frame, data));
    frame.update();
    frame.save()?;

    let mut opd = Png::<f64>::new(results_path.join("oiwfs_correction_opd.png"), 512);
    <OpticalModel<_> as Write<Wavefront>>::write(&mut oiwfs_tt_om)
        .map(|data| <Png<_> as Read<Wavefront>>::read(&mut opd, data));
    opd.update();
    opd.save()?;

    // DFS
    let agws_gs_builder = Source::builder()
        .size(AGWS_N_GS)
        .on_ring(6f32.from_arcmin());

    let dfs_om_builder = OpticalModel::<DFS>::builder()
        .gmt(gmt_builder.clone())
        .source(agws_gs_builder.clone().band("J"))
        .sensor(DFS::builder().source(agws_gs_builder.clone().band("J")));
    let mut offaxis_om: OpticalModel<WaveSensor> =
        OpticalModelBuilder::<WaveSensorBuilder>::from(&dfs_om_builder).build()?;
    println!("{offaxis_om}");

    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut offaxis_om, cmd.clone().into());
    <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(
        &mut offaxis_om,
        m2_bm_e_zm.iter().map(|x| -*x).collect::<Vec<_>>().into(),
    );
    <OpticalModel<_> as Read<M2GlobalTipTilt>>::read(
        &mut offaxis_om,
        vec![-estimate[0], -estimate[1]].into(),
    );
    offaxis_om.update();

    let mut opd = Png::<f64>::new(results_path.join("dfs_opd.png"), 512);
    <OpticalModel<_> as Write<Wavefront>>::write(&mut offaxis_om)
        .map(|data| <Png<_> as Read<Wavefront>>::read(&mut opd, data));
    opd.update();
    opd.save()?;

    let mut recon_rxy = if let Ok(file) = File::open(format!("calib_dfs_m1-rxy.pkl")) {
        serde_pickle::from_reader(file, Default::default())?
    } else {
        let closed_loop_optical_model =
            OpticalModel::<WaveSensor>::builder().gmt(gmt_builder.clone());
        let mut recon_rxy = <DispersedFringeSensorProcessing as ClosedLoopCalibration<
            WaveSensor,
        >>::calibrate_serial(
            &dfs_om_builder,
            MirrorMode::from(CalibrationMode::RBM([
                None,                    // Tx
                None,                    // Ty
                None,                    // Tz
                Some(100f64.from_mas()), // Rx
                Some(100f64.from_mas()), // Ry
                None,                    // Rz
            ]))
            .update((7, CalibrationMode::empty_rbm())),
            &closed_loop_optical_model,
            CalibrationMode::modes(M2_N_MODE, 1e-6),
        )?;
        let mut recon_rxy = recon_rxy.collapse();
        recon_rxy.pseudoinverse();
        serde_pickle::to_writer(
            &mut File::create("calib_dfs_m1-rxy.pkl")?,
            &recon_rxy,
            Default::default(),
        )?;
        recon_rxy
    };
    println!("DFS CALIBRATION:");
    println!("{recon_rxy}");

    // let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
    //     Camera,
    //     M1RigidBodyMotions,
    //     CentroidsProcessing<LTWS>,
    // >>::estimate_with_closed_loop_reconstructor(
    //     &dfs_om_builder,
    //     &ltws_om_builder,
    //     &mut recon_rxy,
    //     &cmd,
    //     &mut calib_m2_modes,
    // )?;
    println!("M1 command");
    cmd.chunks(6).enumerate().for_each(|(i, c)| {
        println!(
            "S{}: {:+7.0?}{:+7.0?}",
            i + 1,
            c[..3].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
            c[3..].iter().map(|x| x.to_mas()).collect::<Vec<_>>()
        )
    });
    // println!("DFS estimation (NGAO)");
    // estimate.chunks(6).enumerate().for_each(|(i, c)| {
    //     println!(
    //         "S{}: {:+7.0?}{:+7.0?}",
    //         i + 1,
    //         c[..3].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
    //         c[3..].iter().map(|x| x.to_mas()).collect::<Vec<_>>()
    //     )
    // });

    let recon2_rxy: Reconstructor<MirrorMode> = serde_pickle::from_reader(
        &mut File::open("examples/dfs_closed-loop_calibration/calib_ltws-oiwfs_dfs_m1-rxy.pkl")?,
        Default::default(),
    )?;

    for (tag, mut recon_rxy) in [("LTWS", recon_rxy), ("LTWS+OIWS", recon2_rxy)] {
        let mut dfs_processor = DispersedFringeSensorProcessing::new();
        dfs_om_builder.initialize(&mut dfs_processor);
        let mut dfs_om = dfs_om_builder.clone().build()?;
        <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut dfs_om, cmd.clone().into());
        <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(
            &mut dfs_om,
            m2_bm_e_zm.iter().map(|x| -*x).collect::<Vec<_>>().into(),
        );
        <OpticalModel<_> as Read<M2GlobalTipTilt>>::read(
            &mut dfs_om,
            vec![-estimate[0], -estimate[1]].into(),
        );
        dfs_om.update();
        <OpticalModel<_> as Write<DfsFftFrame<Dev>>>::write(&mut dfs_om).map(|data| {
            <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(
                &mut dfs_processor,
                data,
            )
        });
        dfs_processor.update();

        <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor)
            .map(|data| <Reconstructor<_, _> as Read<Intercepts>>::read(&mut recon_rxy, data));
        recon_rxy.update();
        let estimate = <Reconstructor<_, _> as Write<M1RigidBodyMotions>>::write(&mut recon_rxy)
            .unwrap()
            .into_arc();
        println!("DFS estimation ({tag})");
        estimate.chunks(6).enumerate().for_each(|(i, c)| {
            println!(
                "S{}: {:+7.0?}{:+7.0?}",
                i + 1,
                c[..3].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
                c[3..].iter().map(|x| x.to_mas()).collect::<Vec<_>>()
            )
        });
    }

    let recon_rxy: Reconstructor<MirrorMode> = serde_pickle::from_reader(
        &mut File::open("examples/dfs_closed-loop_calibration/calib_ltws-oiwfs_dfs_m1-rxy.pkl")?,
        Default::default(),
    )?;
    let recon_tz: Reconstructor<CalibrationMode> = serde_pickle::from_reader(
        &mut File::open("examples/dfs_closed-loop_calibration/calib_dfs_m2-piston.pkl")?,
        Default::default(),
    )?;
    let recon_tz = recon_tz.collapse();
    // let mut split_recon_rxy = recon_rxy.split();
    // println!("{recon_rxy}");
    // println!("{split_recon_rxy}");
    // let split_recon_tz = recon_tz.split();
    println!("{recon_tz}");
    // println!("{split_recon_tz}");
    // split_recon_rxy.merge(recon_tz);
    // let mut recon = split_recon_rxy.collapse();

    let mut recon = Block::block(&[&[&recon_rxy, &recon_tz]]);
    recon.pseudoinverse();
    println!("{recon}");
    let mut dfs_processor = DispersedFringeSensorProcessing::new();
    dfs_om_builder.initialize(&mut dfs_processor);
    let mut dfs_om = dfs_om_builder.clone().build()?;
    <OpticalModel<_> as Read<M1RigidBodyMotions>>::read(&mut dfs_om, cmd.clone().into());
    <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(
        &mut dfs_om,
        m2_bm_e_zm.iter().map(|x| -*x).collect::<Vec<_>>().into(),
    );
    <OpticalModel<_> as Read<M2GlobalTipTilt>>::read(
        &mut dfs_om,
        vec![-estimate[0], -estimate[1]].into(),
    );
    dfs_om.update();
    <OpticalModel<_> as Write<DfsFftFrame<Dev>>>::write(&mut dfs_om).map(|data| {
        <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(&mut dfs_processor, data)
    });
    dfs_processor.update();

    <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor)
        .map(|data| <Reconstructor<_, _> as Read<Intercepts>>::read(&mut recon, data));
    recon.update();
    let estimate = <Reconstructor<_, _> as Write<M1RigidBodyMotions>>::write(&mut recon)
        .unwrap()
        .into_arc();
    dbg!(estimate.len());
    println!("DFS estimation");
    estimate[..42].chunks(6).enumerate().for_each(|(i, c)| {
        println!(
            "S{}: {:+7.0?}{:+7.0?}",
            i + 1,
            c[..3].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
            c[3..].iter().map(|x| x.to_mas()).collect::<Vec<_>>()
        )
    });
    // println!(
    //     "{:?}",
    //     estimate[42..].iter().map(|x| x * 1e9).collect::<Vec<_>>()
    // );
    dbg!(estimate[42..].len());
    estimate[42..]
        .chunks(M2_N_MODE)
        .enumerate()
        .for_each(|(i, c)| {
            println!(
                "S{}: {:+7.0?}",
                i + 1,
                c.iter().take(1).map(|x| x * 1e9).collect::<Vec<_>>(),
                // c[3..].iter().map(|x| x.to_mas()).collect::<Vec<_>>()
            )
        });
    Ok(())
}
