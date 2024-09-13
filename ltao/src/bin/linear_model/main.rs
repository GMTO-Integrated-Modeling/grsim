use gmt_dos_clients_crseo::calibration::Reconstructor;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut calib_m2_modes_onaxis: Reconstructor = serde_pickle::from_reader(
        &mut std::fs::File::open("src/bin/calibration/calib_m2_modes_onaxis.pkl")?,
        Default::default(),
    )?;
    let mut calib_m1_rbm_onaxis: Reconstructor = serde_pickle::from_reader(
        &mut std::fs::File::open("src/bin/calibration/calib_m1_rbm_onaxis.pkl")?,
        Default::default(),
    )?;
    calib_m1_rbm_onaxis.match_areas(&mut calib_m2_modes_onaxis);
    println!("{calib_m2_modes_onaxis}");
    println!("{calib_m1_rbm_onaxis}");

    let m1_to_m2 = calib_m2_modes_onaxis.least_square_solve(&calib_m1_rbm_onaxis);
    m1_to_m2
        .iter()
        .enumerate()
        .for_each(|(i, m1_to_m2)| println!("M1->M2 ({},{})", m1_to_m2.nrows(), m1_to_m2.ncols()));

    let mut calib_m2_modes_offaxis: Reconstructor = serde_pickle::from_reader(
        &mut std::fs::File::open("src/bin/calibration/calib_m2_modes_offaxis.pkl")?,
        Default::default(),
    )?;
    let mut calib_m1_rbm_offaxis: Reconstructor = serde_pickle::from_reader(
        &mut std::fs::File::open("src/bin/calibration/calib_m1_rbm_offaxis.pkl")?,
        Default::default(),
    )?;

    calib_m2_modes_offaxis.match_areas(&mut calib_m1_rbm_offaxis);

    // let m1_to_agws: Vec<_> = calib_m1_rbm_offaxis
    //     .iter()
    //     .zip(calib_m2_modes_offaxis.iter())
    //     .zip(m1_to_m2.iter().map(|x| x.as_ref()))
    //     .map(|((m1_offaxis, m2_off_axis), m1_2_m2)| m1_offaxis - m2_off_axis * m1_2_m2)
    //     .collect();
    // m1_to_agws.iter().for_each(|m1_to_agws| {
    //     println!("M1->AGWS ({},{})", m1_to_agws.nrows(), m1_to_agws.ncols())
    // });
    let mut m1_to_agws = &mut calib_m1_rbm_offaxis;
    m1_to_agws -= &calib_m2_modes_offaxis * m1_to_m2;
    m1_to_agws.pseudoinverse();
    println!("{m1_to_agws}");

    serde_pickle::to_writer(
        &mut std::fs::File::create("src/bin/linear_model/m1_to_agws.pkl")?,
        &m1_to_agws,
        Default::default(),
    )?;

    Ok(())
}
