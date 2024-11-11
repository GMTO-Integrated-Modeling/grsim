use std::marker::PhantomData;

use crseo::{
    atmosphere::AtmosphereBuilder, gmt::GmtBuilder, source::SourceBuilder, FromBuilder, Gmt, Source,
};
use gmt_dos_clients_crseo::{
    sensors::{builders::SensorBuilderProperty, SensorPropagation},
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
};
use skyangle::Conversion;

pub const M1_N_MODE: usize = 27;
pub const M2_N_MODE: usize = 66;
const AGWS_N_GS: usize = 3;

pub trait Model: ModelBuilder
where
    OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<Self::Processor>,
{
    type Sensor: FromBuilder;
    type Processor;
    type Estimator;
    fn processor(&self) -> anyhow::Result<Self::Processor>;
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>;
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator>;
    fn atmosphere(&self) -> Option<AtmosphereBuilder> {
        None
    }
    fn build(&self) -> anyhow::Result<
        OpticalModel<
            <<<Self as Model>::Sensor as FromBuilder>::ComponentBuilder as crseo::Builder>::Component,
        >,
    >
{
        <Self as ModelBuilder>::model_build(self)
    }
}

pub trait ModelBuilder {
    fn model_build(
        &self,
    ) -> anyhow::Result<
        OpticalModel<
            <<<Self as Model>::Sensor as FromBuilder>::ComponentBuilder as crseo::Builder>::Component,
        >,
    >
    where
        Self: Model,
        OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>:
            DeviceInitialize<Self::Processor>;
}
impl<T> ModelBuilder for T
where
    T: Model,
    <<T as Model>::Sensor as FromBuilder>::ComponentBuilder: SensorBuilderProperty,
    <<<T as Model>::Sensor as FromBuilder>::ComponentBuilder as crseo::Builder>::Component:
        SensorPropagation,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
{
    fn model_build(&self) -> anyhow::Result<
        OpticalModel<
            <<<Self as Model>::Sensor as FromBuilder>::ComponentBuilder as crseo::Builder>::Component,
        >,
    >
    {
        // Ok(self.builder().build()?)
        Ok(if let Some(atmosphere) = <T as Model>::atmosphere(self) {
            <T as Model>::builder(self).atmosphere(atmosphere)
        } else {
            <T as Model>::builder(self)
        }
        .build()?)
    }
}

#[derive(Debug, Default, Clone)]
pub struct Models {
    pub gmt_builder: GmtBuilder,
    pub agws_gss: SourceBuilder,
    pub atm_builder: Option<AtmosphereBuilder>,
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
            atm_builder: None,
        }
    }
    pub fn atmosphere(mut self, atm: AtmosphereBuilder) -> Self {
        self.atm_builder = Some(atm);
        self
    }
    pub fn gmt(&self) -> GmtBuilder {
        self.gmt_builder.clone()
    }
    pub fn ltws(&self) -> Ltws {
        Ltws(self.clone())
    }
    pub fn sh48<const C: usize>(&self) -> Sh48<C> {
        Sh48(Self {
            agws_gss: self
                .agws_gss
                .clone()
                .pupil_size(48 as f64 * 0.53)
                .band("R")
                .fwhm(6.),
            ..self.clone()
        })
    }
    pub fn oiwfs(&self) -> Oiwfs {
        Oiwfs(self.clone())
    }
    pub fn dfs<M: DfsModes, const C: usize, const F: usize>(&self) -> Dfs<M, C, F> {
        Dfs(
            Self {
                agws_gss: self.agws_gss.clone().band("J"),
                ..self.clone()
            },
            PhantomData,
        )
    }
}

mod dfs;
mod ltws;
mod m2_gtt_to_ptt;
mod m2_merge_cmd;
mod oiwfs;
mod sh48;
pub use dfs::{Dfs, DfsModes, Rxy, RxyPiston};
pub use ltws::Ltws;
pub use m2_gtt_to_ptt::M2GttToPtt;
pub use m2_merge_cmd::MergeAsmCommand;
pub use oiwfs::Oiwfs;
pub use sh48::Sh48;
