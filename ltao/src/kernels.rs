use std::{marker::PhantomData, ops::Deref};

use crseo::FromBuilder;
use gmt_dos_clients_crseo::{DeviceInitialize, OpticalModel, OpticalModelBuilder};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

use crate::{Model, Models};

pub struct KernelFrame<T>(PhantomData<T>)
where
    T: Model + KernelSpecs + Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>;
impl<T> UniqueIdentifier for KernelFrame<T>
where
    T: Model + KernelSpecs + Deref<Target = Models> + Send + Sync,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
    <T as KernelSpecs>::Input: UniqueIdentifier,
{
    type DataType = <<T as KernelSpecs>::Input as UniqueIdentifier>::DataType;
}
impl<T> Write<KernelFrame<T>> for OpticalModel<<T as Model>::Sensor>
where
    T: Model + KernelSpecs + Deref<Target = Models> + Send + Sync,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
    KernelFrame<T>:
        UniqueIdentifier<DataType = <<T as KernelSpecs>::Input as UniqueIdentifier>::DataType>,
    <T as KernelSpecs>::Input: UniqueIdentifier,
    Self: Write<<T as KernelSpecs>::Input>,
{
    fn write(&mut self) -> Option<Data<KernelFrame<T>>> {
        <Self as Write<<T as KernelSpecs>::Input>>::write(self)
            .map(|data| data.transmute::<KernelFrame<T>>())
    }
}

pub trait KernelSpecs {
    type Integrator;
    type Input: Send + Sync;
    type Data: Send + Sync;
    type Output: Send + Sync;
}

pub struct Kernel<T>
where
    T: Model + KernelSpecs + Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
{
    processor: <T as Model>::Processor,
    estimator: <T as Model>::Estimator,
    integrator: <T as KernelSpecs>::Integrator,
}
impl<T> Kernel<T>
where
    T: Model + KernelSpecs + Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
{
    pub fn new(model: &T, integrator: <T as KernelSpecs>::Integrator) -> anyhow::Result<Self> {
        Ok(Self {
            processor: <T as Model>::processor(model)?,
            estimator: <T as Model>::reconstructor(model)?,
            integrator,
        })
    }
}

impl<T> Read<<T as KernelSpecs>::Input> for Kernel<T>
where
    T: Model + KernelSpecs + Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
    <T as KernelSpecs>::Input: UniqueIdentifier,
    <T as Model>::Processor: Read<<T as KernelSpecs>::Input>,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as Model>::Processor: Write<<T as KernelSpecs>::Data>,
    <T as Model>::Estimator: Read<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as Model>::Estimator: Write<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: Read<<T as KernelSpecs>::Output>,
{
    fn read(&mut self, data: Data<<T as KernelSpecs>::Input>) {
        <<T as Model>::Processor as Read<_>>::read(&mut self.processor, data);
    }
}
impl<T> Update for Kernel<T>
where
    T: Model + KernelSpecs + Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as Model>::Processor: Write<<T as KernelSpecs>::Data>,
    <T as Model>::Estimator: Read<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as Model>::Estimator: Write<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: Read<<T as KernelSpecs>::Output>,
{
    fn update(&mut self) {
        self.processor.update();
        <<T as Model>::Processor as Write<<T as KernelSpecs>::Data>>::write(&mut self.processor)
            .map(|data| {
                <<T as Model>::Estimator as Read<<T as KernelSpecs>::Data>>::read(
                    &mut self.estimator,
                    data,
                )
            });
        self.estimator.update();
        <<T as Model>::Estimator as Write<<T as KernelSpecs>::Output>>::write(&mut self.estimator)
            .map(|data| {
                <<T as KernelSpecs>::Integrator as Read<<T as KernelSpecs>::Output>>::read(
                    &mut self.integrator,
                    data,
                )
            });
        self.integrator.update();
    }
}
impl<T> Write<<T as KernelSpecs>::Output> for Kernel<T>
where
    T: Model + KernelSpecs + Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Integrator: Write<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as Model>::Processor: Write<<T as KernelSpecs>::Data>,
    <T as Model>::Estimator: Read<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as Model>::Estimator: Write<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: Read<<T as KernelSpecs>::Output>,
{
    fn write(&mut self) -> Option<Data<<T as KernelSpecs>::Output>> {
        <<T as KernelSpecs>::Integrator as Write<_>>::write(&mut self.integrator)
    }
}
