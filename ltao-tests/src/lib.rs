use std::marker::PhantomData;

use crseo::{gmt::GmtBuilder, source::SourceBuilder, FromBuilder, Gmt, Source};
use gmt_dos_clients_crseo::{DeviceInitialize, OpticalModel, OpticalModelBuilder};
use skyangle::Conversion;

pub const M1_N_MODE: usize = 27;
pub const M2_N_MODE: usize = 66;
const AGWS_N_GS: usize = 3;

pub struct Ltws {
    gmt_builder: GmtBuilder,
}
pub struct Oiwfs {
    gmt_builder: GmtBuilder,
}

pub enum Rxy {}
pub enum RxyPiston {}
pub trait DfsModes {}
impl DfsModes for Rxy {}
impl DfsModes for RxyPiston {}
impl DfsModes for () {}

pub struct Dfs<M, const C: usize, const F: usize>
where
    M: DfsModes,
{
    gmt_builder: GmtBuilder,
    agws_gss: SourceBuilder,
    modes: PhantomData<M>,
}

pub struct Sh48<const C: usize> {
    gmt_builder: GmtBuilder,
    agws_gss: SourceBuilder,
}

pub trait Model
where
    OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<Self::Processor>,
{
    type Sensor: FromBuilder;
    type Processor;
    type Estimator;
    fn processor(&self) -> anyhow::Result<Self::Processor>;
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>;
    fn build(&self) -> anyhow::Result<OpticalModel<Self::Sensor>>;
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator>;
}

pub struct Models {
    pub gmt_builder: GmtBuilder,
    pub agws_gss: SourceBuilder,
}

impl Models {
    pub fn new() -> Self where {
        Self {
            gmt_builder: Gmt::builder()
                .m1("bending modes", M1_N_MODE)
                .m2("Karhunen-Loeve", M2_N_MODE)
                .m1_truss_projection(false),
            agws_gss: Source::builder()
                .size(AGWS_N_GS)
                .on_ring(6f32.from_arcmin()),
        }
    }
    pub fn gmt(&self) -> GmtBuilder {
        self.gmt_builder.clone()
    }
    pub fn ltws(&self) -> Ltws {
        Ltws {
            gmt_builder: self.gmt_builder.clone(),
        }
    }
    pub fn sh48<const C: usize>(&self) -> Sh48<C> {
        Sh48 {
            gmt_builder: self.gmt_builder.clone(),
            agws_gss: self
                .agws_gss
                .clone()
                .pupil_size(48 as f64 * 0.53)
                .band("R")
                .fwhm(6.),
        }
    }
    pub fn oiwfs(&self) -> Oiwfs {
        Oiwfs {
            gmt_builder: self.gmt_builder.clone(),
        }
    }
    pub fn dfs<M: DfsModes, const C: usize, const F: usize>(&self) -> Dfs<M, C, F> {
        Dfs {
            gmt_builder: self.gmt_builder.clone(),
            agws_gss: self.agws_gss.clone().band("J"),
            modes: PhantomData,
        }
    }
}

mod dfs;
mod ltws;
mod m2_gtt_to_ptt;
mod m2_merge_cmd;
mod oiwfs;
mod sh48;
pub use m2_gtt_to_ptt::M2GttToPtt;
pub use m2_merge_cmd::MergeAsmCommand;
