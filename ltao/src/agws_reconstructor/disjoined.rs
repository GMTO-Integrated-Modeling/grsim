use std::fs::File;

use gmt_dos_clients_crseo::calibration::ClosedLoopReconstructor;
use gmt_dos_clients_io::{
    gmt_m1::{M1ModeShapes, M1RigidBodyMotions},
    optics::dispersed_fringe_sensor::Intercepts,
};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

use super::{AgwsReconstructor, AgwsSh48, Disjoined};

impl AgwsReconstructor<Disjoined> {
    pub fn new(m1_n_mode: usize) -> anyhow::Result<Self> {
        let sh48_recon: ClosedLoopReconstructor = serde_pickle::from_reader(
            File::open(format!("src/bin/agws_oiwfs/calib_sh48_{m1_n_mode}bm.pkl"))?,
            Default::default(),
        )?;
        let dfs_recon: ClosedLoopReconstructor = serde_pickle::from_reader(
            File::open("src/bin/dfs_calibration/calib_dfs_closed-loop_m1-rxy_v2.pkl")?,
            Default::default(),
        )?;
        Ok(Self {
            sh48_recon,
            dfs_recon,
            ..Default::default()
        })
    }
}

impl Update for AgwsReconstructor<Disjoined> {
    fn update(&mut self) {
        <ClosedLoopReconstructor as Update>::update(&mut self.sh48_recon);
        <ClosedLoopReconstructor as Update>::update(&mut self.dfs_recon);
    }
}

impl Read<Intercepts> for AgwsReconstructor<Disjoined> {
    fn read(&mut self, data: Data<Intercepts>) {
        <ClosedLoopReconstructor as Read<Intercepts>>::read(&mut self.dfs_recon, data);
    }
}

impl<U> Read<U> for AgwsReconstructor<Disjoined>
where
    U: UniqueIdentifier<DataType = Vec<f64>> + AgwsSh48,
{
    fn read(&mut self, data: Data<U>) {
        <ClosedLoopReconstructor as Read<U>>::read(&mut self.sh48_recon, data);
    }
}

impl Write<M1RigidBodyMotions> for AgwsReconstructor<Disjoined> {
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        <ClosedLoopReconstructor as Write<M1RigidBodyMotions>>::write(&mut self.dfs_recon).map(
            |data| {
                data.as_arc()
                    .iter()
                    .cloned()
                    .chain(vec![0f64; 6])
                    .collect::<Vec<_>>()
                    .into()
            },
        )
    }
}
impl Write<M1ModeShapes> for AgwsReconstructor<Disjoined> {
    fn write(&mut self) -> Option<Data<M1ModeShapes>> {
        <ClosedLoopReconstructor as Write<M1ModeShapes>>::write(&mut self.sh48_recon)
    }
}
