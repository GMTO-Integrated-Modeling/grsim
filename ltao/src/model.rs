use std::{marker::PhantomData, ops::Deref};

use crseo::{
    atmosphere::AtmosphereBuilder, gmt::GmtBuilder, source::SourceBuilder, FromBuilder, Gmt, Source,
};
use gmt_dos_clients_crseo::{
    sensors::{builders::SensorBuilderProperty, SensorPropagation},
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
};
use skyangle::Conversion;

mod dfs;
mod ltws;
mod oiwfs;
mod sh48;

pub use dfs::{Dfs, DfsModes, Rxy, RxyPiston};
pub use ltws::Ltws;
pub use oiwfs::Oiwfs;
pub use sh48::Sh48;

use crate::{
    agws_parameters::AGWS_N_GS,
    kernels::{Kernel, KernelSpecs},
    m1_parameters::{M1_N_MODE, M1_TRUSS_PROJECTION, RAW_BENDING_MODES},
    m2_parameters::{ASMS_INFLUENCE_FUNCTIONS, ASM_N_ACTUATOR},
};

pub trait Model: ModelBuilder + KernelSpecs
where
    Self: Deref<Target = Models>,
    OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<Self::Processor>,
{
    type Sensor: FromBuilder;
    type Processor;
    type Estimator;
    fn processor(&self) -> anyhow::Result<Self::Processor>;
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>;
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator>;
    fn atmosphere(&self) -> Option<(f64, AtmosphereBuilder)> {
        <Self as Deref>::deref(self).atm_builder.clone()
    }
    fn kernel(&self, integrator: <Self as KernelSpecs>::Integrator) -> anyhow::Result<Kernel<Self>>
    where
        Self: Sized,
    {
        Kernel::new(self, integrator)
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
        Ok(
            if let Some((sampling_frequency, atmosphere)) = <T as Model>::atmosphere(self) {
                <T as Model>::builder(self)
                    .sampling_frequency(sampling_frequency)
                    .atmosphere(atmosphere)
            } else {
                <T as Model>::builder(self)
            }
            .build()?,
        )
    }
}

#[derive(Debug, Default, Clone)]
pub struct Models {
    pub gmt_builder: GmtBuilder,
    pub agws_gss: SourceBuilder,
    pub atm_builder: Option<(f64, AtmosphereBuilder)>,
}

impl Models {
    pub fn new() -> Self where {
        Self {
            gmt_builder: Gmt::builder()
                .m1(RAW_BENDING_MODES, M1_N_MODE)
                .m2(ASMS_INFLUENCE_FUNCTIONS, ASM_N_ACTUATOR)
                .m1_truss_projection(M1_TRUSS_PROJECTION),
            agws_gss: Source::builder()
                .size(AGWS_N_GS)
                .on_ring(6f32.from_arcmin()),
            ..Default::default()
        }
    }
    pub fn atmosphere(mut self, sampling_frequency: f64, atm: AtmosphereBuilder) -> Self {
        self.atm_builder = Some((sampling_frequency, atm));
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
            agws_gss: self.agws_gss.clone().pupil_size(48_f64 * 0.53).band("R"),
            // .fwhm(6.),
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
