use std::{ops::{Mul, Deref, DerefMut}, sync::Arc};

use crseo::CrseoError;
use gmt_dos_clients_crseo::DetectorFrame;
use interface::{Data, Read, UniqueIdentifier, Update, Write};

mod pyramid;
pub use pyramid::{PyramidCalibrator, PyramidCommand, PyramidMeasurements, PyramidProcessor};

/// Sensor data processing interface
pub trait Processing {
    type ProcessorData;
    fn processing(&self) -> Self::ProcessorData;
}

/// Sensor data processor
#[derive(Default, Debug)]
pub struct Processor<P: Processing>(P);

impl<P: Processing> From<P> for Processor<P> {
    fn from(value: P) -> Self {
        Processor(value)
    }
}

impl<P: Processing> Deref for Processor<P> {
    type Target = P;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<P: Processing> DerefMut for Processor<P> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<P: Processing + Send + Sync> Update for Processor<P> {
    // fn update(&mut self) {
    //     self.processing();
    // }
}

impl Read<DetectorFrame<f32>> for Processor<PyramidProcessor<f32>> {
    fn read(&mut self, data: Data<DetectorFrame<f32>>) {
        self.frame = data.as_arc();
    }
}

impl<P, T> Write<T> for Processor<P>
where
    P: Processing + Send + Sync,
    T: UniqueIdentifier<DataType = P::ProcessorData>,
{
    fn write(&mut self) -> Option<Data<T>> {
        let data: <P as Processing>::ProcessorData = self.processing();
        Some(Data::new(data))
    }
}

#[derive(Debug, thiserror::Error)]
pub enum CalibratingError {
    #[error("crseo error")]
    Crseo(#[from] CrseoError),
}

/// Sensor calibration interface
pub trait Calibrating {
    type ProcessorData: Default;
    type Output;
    // type Calibrator;
    // fn calibrating(&self) -> Result<Self::Calibrator, CalibratingError>;
}

/// Sensor calibration
pub struct Calibration<C: Calibrating> {
    calibrator: C,
    output: Arc<C::Output>,
}

impl<C: Calibrating> Deref for Calibration<C> {
    type Target = C;

    fn deref(&self) -> &Self::Target {
        &self.calibrator
    }
}

impl<C: Calibrating> DerefMut for Calibration<C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.calibrator
    }
}

impl<C: Calibrating + Send + Sync> Update for Calibration<C>
where
    <C as Calibrating>::ProcessorData: Sync + Send,
    <C as Calibrating>::Output: Send + Sync,
    // for<'a> &'a C: Mul<&'a C::ProcessorData, Output = ()>,
{
    // fn update(&mut self) {
    //     &self.calibrator * &self.data
    // }
}

impl<C: Calibrating + Send + Sync, T: UniqueIdentifier<DataType = C::ProcessorData>> Read<T>
    for Calibration<C>
where
    <C as Calibrating>::ProcessorData: Send + Sync,
    <C as Calibrating>::Output: Send + Sync,
    for<'a> &'a C: Mul<&'a C::ProcessorData, Output = <C as Calibrating>::Output>,
{
    fn read(&mut self, data: Data<T>) {
        let value = data.as_arc();
        self.output = Arc::new(&self.calibrator * &value);
    }
}

impl<C: Calibrating + Send + Sync, T: UniqueIdentifier<DataType = C::Output>> Write<T>
    for Calibration<C>
where
    <C as Calibrating>::ProcessorData: Send + Sync,
    <C as Calibrating>::Output: Send + Sync,
{
    fn write(&mut self) -> Option<Data<T>> {
        Some(Data::from(&self.output))
    }
}
