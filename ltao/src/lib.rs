use std::{fs::File, sync::Arc};

use gmt_dos_clients_crseo::calibration::Reconstructor;
use gmt_dos_clients_io::{
    gmt_m1::{M1ModeShapes, M1RigidBodyMotions},
    optics::{dispersed_fringe_sensor::Intercepts, SensorData},
};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

#[derive(Default, Debug)]
pub struct MergedReconstructor {
    recon: Reconstructor,
    intercepts: Arc<Vec<f64>>,
    centroids_mask: Vec<bool>,
    centroids: Vec<f64>,
    rxy: Vec<f64>,
    bm: Vec<f64>,
}

impl MergedReconstructor {
    pub fn new() -> Self {
        let (recon, centroids_mask): (Reconstructor, Vec<bool>) = serde_pickle::from_reader(
            File::open("src/bin/merged_reconstructor/merged-reconstructor.pkl").unwrap(),
            Default::default(),
        )
        .unwrap();
        println!("{recon}");
        Self {
            recon,
            centroids_mask,
            ..Default::default()
        }
    }
}
impl Update for MergedReconstructor {
    fn update(&mut self) {
        let y: Vec<_> = self
            .centroids
            .iter()
            .cloned()
            .chain(self.intercepts.iter().cloned())
            .collect();
        let c = faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1) / &self.recon;
        self.rxy = c[0].col_as_slice(0)[..14].to_vec();
        self.bm = c[0].col_as_slice(0)[14..].to_vec();
    }
}

impl Read<Intercepts> for MergedReconstructor {
    fn read(&mut self, data: Data<Intercepts>) {
        self.intercepts = data.into_arc();
    }
}

pub trait MergedReconstructorSh48 {}

impl<U: UniqueIdentifier<DataType = Vec<f64>> + MergedReconstructorSh48> Read<U>
    for MergedReconstructor
{
    fn read(&mut self, data: Data<U>) {
        self.centroids = data
            .iter()
            .cycle()
            .zip(self.centroids_mask.iter())
            .filter_map(|(x, b)| if *b { Some(*x) } else { None })
            .collect();
    }
}

impl Write<M1RigidBodyMotions> for MergedReconstructor {
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        let mut m1_rbm: Vec<f64> = self
            .rxy
            .chunks(2)
            .flat_map(|rxy| {
                vec![0.; 3]
                    .into_iter()
                    .chain(rxy.to_vec().into_iter())
                    .chain(Some(0.))
                    .collect::<Vec<_>>()
            })
            .collect();
        Some(m1_rbm.into())
    }
}
impl Write<M1ModeShapes> for MergedReconstructor {
    fn write(&mut self) -> Option<Data<M1ModeShapes>> {
        Some(self.bm.clone().into())
    }
}
