use std::fs::File;

use gmt_dos_clients_crseo::calibration::{
    algebra::CalibProps, MixedMirrorMode, Reconstructor,
};
use gmt_dos_clients_io::{
    gmt_m1::{M1ModeShapes, M1RigidBodyMotions},
    optics::dispersed_fringe_sensor::Intercepts,
};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

use super::{AgwsReconstructor, AgwsSh48, Merged};

impl AgwsReconstructor<Merged> {
    pub fn new() -> anyhow::Result<Self> {
        let (recon, norm_dfs, norm_sh48): (Reconstructor<MixedMirrorMode>, Vec<f64>, Vec<f64>) =
            serde_pickle::from_reader(
                File::open("src/bin/merged_reconstructor/merged_recon_fqp.pkl")?,
                Default::default(),
            )?;
        println!("{recon}");
        Ok(Self {
            recon,
            norm_dfs: norm_dfs[0],
            norm_sh48: norm_sh48[0],
            ..Default::default()
        })
    }
}

impl Update for AgwsReconstructor<Merged> {
    fn update(&mut self) {
        let y: Vec<_> = self
            .intercepts
            .iter()
            .map(|x| *x / self.norm_dfs)
            .chain(self.centroids.iter().map(|x| x / self.norm_sh48))
            .collect();
        let c = faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1) / &self.recon;
        self.rbm = c[0].col_as_slice(0)[..40].to_vec();
        self.rbm.insert(38, 0.);
        self.rbm.insert(41, 0.);
        self.bm = c[0].col_as_slice(0)[40..].to_vec();
    }
}

impl Read<Intercepts> for AgwsReconstructor<Merged> {
    fn read(&mut self, data: Data<Intercepts>) {
        self.intercepts = data.into_arc();
    }
}

impl<U> Read<U> for AgwsReconstructor<Merged>
where
    U: UniqueIdentifier<DataType = Vec<f64>> + AgwsSh48,
{
    fn read(&mut self, data: Data<U>) {
        let m = &self.recon.calib_slice()[0].mask_as_slice()[36..];
        self.centroids = data
            .iter()
            .cycle()
            .zip(m)
            .filter_map(|(x, b)| if *b { Some(*x) } else { None })
            .collect();
    }
}

impl Write<M1RigidBodyMotions> for AgwsReconstructor<Merged> {
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        Some(self.rbm.clone().into())
    }
}
impl Write<M1ModeShapes> for AgwsReconstructor<Merged> {
    fn write(&mut self) -> Option<Data<M1ModeShapes>> {
        Some(self.bm.clone().into())
    }
}
