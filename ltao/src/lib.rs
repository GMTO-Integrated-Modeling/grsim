use gmt_dos_clients_crseo::{
    sensors::{Camera, WaveSensor},
    OpticalModel,
};
use gmt_dos_clients_io::optics::Wavefront;
use interface::UID;

pub mod agws_reconstructor;
pub use agws_reconstructor::AgwsReconstructor;
use agws_reconstructor::AgwsSh48;

#[derive(UID)]
#[uid(port = 50_555)]
pub enum M1ModesNorm {}

#[derive(UID)]
#[uid(port = 50_556)]
pub enum M1Rxy {}

#[derive(UID)]
#[alias(name=Wavefront, client=OpticalModel<WaveSensor>, traits=Write,Size)]
pub enum DfsWavefront {}

#[derive(UID)]
#[alias(name=Wavefront, client=OpticalModel<Camera<1>>, traits=Write,Size)]
pub enum OiwfsWavefront {}

#[derive(UID)]
pub enum LtwsData {}

#[derive(UID)]
pub enum Sh48Data {}
impl AgwsSh48 for Sh48Data {}

#[derive(UID)]
pub enum LtwsResidualAsmCmd {}

#[derive(UID)]
pub enum OiwfsResidualAsmCmd {}

#[derive(UID)]
pub enum OiwfsData {}

const OIWFS: usize = 1;

pub mod agws;
pub use agws::Agws;
