use std::{marker::PhantomData, sync::Arc};

use gmt_dos_clients_crseo::calibration::{
    ClosedLoopReconstructor, MirrorMode, MixedMirrorMode, Reconstructor,
};

#[derive(Default)]
pub struct Merged {}
#[derive(Default)]
pub struct Disjoined {}

pub trait AgwsReconstructorKind {}

impl AgwsReconstructorKind for Merged {}
impl AgwsReconstructorKind for Disjoined {}

pub trait AgwsSh48 {}

#[derive(Default, Debug)]
pub struct AgwsReconstructor<K = Disjoined>
where
    K: AgwsReconstructorKind,
{
    recon: Reconstructor<MixedMirrorMode>,
    sh48_recon: ClosedLoopReconstructor,
    dfs_recon: ClosedLoopReconstructor,
    intercepts: Arc<Vec<f64>>,
    centroids_mask: Vec<bool>,
    centroids: Vec<f64>,
    rbm: Vec<f64>,
    rxy: Vec<f64>,
    bm: Vec<f64>,
    kind: PhantomData<K>,
    norm_dfs: f64,
    norm_sh48: f64,
}

mod disjoined;
mod merged;
