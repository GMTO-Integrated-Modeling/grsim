use std::ops::{Deref, DerefMut};

use gmt_dos_clients_crseo::{
    sensors::{Camera, DispersedFringeSensor},
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

use crate::{Model, Models};

pub trait OpticalModels {}

pub struct LtwsOpticalModel<const LTWS_RATE: usize = 1>(pub(crate) OpticalModel<Camera<LTWS_RATE>>);
impl<const LTWS_RATE: usize> OpticalModels for LtwsOpticalModel<LTWS_RATE> {}
impl<const LTWS_RATE: usize> Deref for LtwsOpticalModel<LTWS_RATE> {
    type Target = OpticalModel<Camera<LTWS_RATE>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const LTWS_RATE: usize> DerefMut for LtwsOpticalModel<LTWS_RATE> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const LTWS_RATE: usize> Update for LtwsOpticalModel<LTWS_RATE> {
    fn update(&mut self) {
        self.0.update();
    }
}
pub struct OiwfsOpticalModel<const OIWFS_RATE: usize = 1>(
    pub(crate) OpticalModel<Camera<OIWFS_RATE>>,
);
impl<const OIWFS_RATE: usize> OpticalModels for OiwfsOpticalModel<OIWFS_RATE> {}
impl<const OIWFS_RATE: usize> Deref for OiwfsOpticalModel<OIWFS_RATE> {
    type Target = OpticalModel<Camera<OIWFS_RATE>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const OIWFS_RATE: usize> DerefMut for OiwfsOpticalModel<OIWFS_RATE> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const OIWFS_RATE: usize> Update for OiwfsOpticalModel<OIWFS_RATE> {
    fn update(&mut self) {
        self.0.update();
    }
}
pub struct Sh48OpticalModel<const SH48_RATE: usize>(pub(crate) OpticalModel<Camera<SH48_RATE>>);
impl<const SH48_RATE: usize> OpticalModels for Sh48OpticalModel<SH48_RATE> {}
impl<const SH48_RATE: usize> Deref for Sh48OpticalModel<SH48_RATE> {
    type Target = OpticalModel<Camera<SH48_RATE>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const SH48_RATE: usize> DerefMut for Sh48OpticalModel<SH48_RATE> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const SH48_RATE: usize> Update for Sh48OpticalModel<SH48_RATE> {
    fn update(&mut self) {
        self.0.update();
    }
}
pub struct DfsOpticalModel<const DFS_CAM_RATE: usize, const DFS_FFT_RATE: usize>(
    pub(crate) OpticalModel<DispersedFringeSensor<DFS_CAM_RATE, DFS_FFT_RATE>>,
);
impl<const DFS_CAM_RATE: usize, const DFS_FFT_RATE: usize> OpticalModels
    for DfsOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE>
{
}
impl<const DFS_CAM_RATE: usize, const DFS_FFT_RATE: usize> Deref
    for DfsOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE>
{
    type Target = OpticalModel<DispersedFringeSensor<DFS_CAM_RATE, DFS_FFT_RATE>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const DFS_CAM_RATE: usize, const DFS_FFT_RATE: usize> DerefMut
    for DfsOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const DFS_CAM_RATE: usize, const DFS_FFT_RATE: usize> Update
    for DfsOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE>
{
    fn update(&mut self) {
        self.0.update();
    }
}

// impl<T: OpticalModels +Deref + DerefMut, U: UniqueIdentifier> Read<U> for T {
//     fn read(&mut self, data: Data<U>) {

//         <<T as Deref>::Target> as Read<U>>::read(&mut self.deref_mut(),data)
//     }
// }

pub struct Meta<T>(
    pub(crate) OpticalModel<
        <<<T as Model>::Sensor as crseo::FromBuilder>::ComponentBuilder as crseo::Builder>::Component,
    >,
) where T: Model
,   T : Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as crseo::FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>;
impl<T> Deref for Meta<T>
where
    T: Model,
    T: Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as crseo::FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
{
    type Target=OpticalModel<
        <<<T as Model>::Sensor as crseo::FromBuilder>::ComponentBuilder as crseo::Builder>::Component,
    >;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<T> DerefMut for Meta<T>
where
    T: Model,
    T: Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as crseo::FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T> Update for Meta<T>
where
    T: Model,
    T: Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as crseo::FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
    OpticalModel<
        <<<T as Model>::Sensor as crseo::FromBuilder>::ComponentBuilder as crseo::Builder>::Component,
    >:Update{
    fn update(&mut self) {
        self.0.update()
    }
}
impl<T,U> Read<U> for Meta<T>
where
    T: Model,
    T: Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as crseo::FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
    OpticalModel<
        <<<T as Model>::Sensor as crseo::FromBuilder>::ComponentBuilder as crseo::Builder>::Component,
    >:Update+Read<U>,
U:UniqueIdentifier
{
    fn read(&mut self, data: Data<U>) {
    <OpticalModel<
        <<<T as Model>::Sensor as crseo::FromBuilder>::ComponentBuilder as crseo::Builder>::Component
    > as Read<U>>::read(&mut self.0, data);
    }
    
}
impl<T,U> Write<U> for Meta<T>
where
    T: Model,
    T: Deref<Target = Models>,
    OpticalModelBuilder<<T::Sensor as crseo::FromBuilder>::ComponentBuilder>:
        DeviceInitialize<T::Processor>,
    OpticalModel<
        <<<T as Model>::Sensor as crseo::FromBuilder>::ComponentBuilder as crseo::Builder>::Component,
    >:Update+Write<U>,
U:UniqueIdentifier
{
    fn write(&mut self) -> Option<Data<U>> {
    <OpticalModel<
        <<<T as Model>::Sensor as crseo::FromBuilder>::ComponentBuilder as crseo::Builder>::Component
    > as Write<U>>::write(&mut self.0 )
    }
}
