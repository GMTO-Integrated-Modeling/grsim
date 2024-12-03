use gmt_dos_clients_crseo::{
    calibration::{ClosedLoopCalibration, ClosedLoopReconstructor},
    sensors::{builders::CameraBuilder, WaveSensor},
    DeviceInitialize,
};
use gmt_dos_clients_io::{
    gmt_m1::M1ModeShapes,
    optics::{Dev, Frame, SensorData},
};
use std::{
    fs::File,
    ops::{Deref, DerefMut},
};

use crseo::{
    imaging::{Detector, LensletArray},
    FromBuilder,
};
use gmt_dos_clients_crseo::{
    calibration::CalibrationMode, centroiding::CentroidsProcessing, sensors::Camera, OpticalModel,
    OpticalModelBuilder,
};

use crate::{kernels::KernelSpecs, Model, Models, M1_N_MODE, M2_N_MODE};

pub struct Sh48<const C: usize>(pub(crate) Models);
impl<const C: usize> Deref for Sh48<C> {
    type Target = Models;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const C: usize> DerefMut for Sh48<C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl<const C: usize> KernelSpecs for Sh48<C> {
    type Integrator = gmt_dos_clients::Integrator<M1ModeShapes>;
    type Input = Frame<Dev>;
    type Data = SensorData;
    type Output = M1ModeShapes;
}
impl<const C: usize> Sh48<C> {
    pub fn sh48(&self) -> CameraBuilder<C> {
        Camera::<C>::builder()
            .lenslet_array(
                LensletArray::default()
                    .n_side_lenslet(48)
                    .n_px_lenslet(24)
                    .pitch(0.53),
            )
            .detector(Detector::default().n_px_framelet(8))
            .lenslet_flux(0.75)
            .n_sensor(self.agws_gss.size)
    }
    pub fn fwhm(mut self, fwhm: f64) -> Self {
        self.0.agws_gss = self.0.agws_gss.fwhm(fwhm);
        self
    }
}

impl<const C: usize> Model for Sh48<C> {
    type Sensor = Camera<C>;
    type Processor = CentroidsProcessing;
    type Estimator = ClosedLoopReconstructor;
    fn processor(&self) -> anyhow::Result<Self::Processor> {
        let mut centroids = CentroidsProcessing::try_from(&self.sh48())?;
        self.builder()
            .source(self.agws_gss.clone().fwhm(6.))
            .initialize(&mut centroids);
        Ok(centroids)
    }
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder> {
        OpticalModel::<Camera<C>>::builder()
            .gmt(self.gmt_builder.clone())
            .source(self.agws_gss.clone())
            .sensor(self.sh48())
    }
    // fn build(&self) -> anyhow::Result<OpticalModel<Self::Sensor>> {
    //     Ok(self.builder().build()?)
    // }
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator> {
        let calib_sh48_bm: ClosedLoopReconstructor =
            if let Ok(file) = File::open(format!("calib_sh48_{M1_N_MODE}bm.pkl")) {
                serde_pickle::from_reader(file, Default::default())?
            } else {
                let closed_loop_optical_model =
                    OpticalModel::<WaveSensor>::builder().gmt(self.gmt_builder.clone());
                let mut calib_sh48_bm =
                    <CentroidsProcessing as ClosedLoopCalibration<WaveSensor>>::calibrate(
                        &self.builder().source(self.agws_gss.clone().fwhm(6.)).into(),
                        CalibrationMode::modes(M1_N_MODE, 1e-4),
                        &closed_loop_optical_model,
                        CalibrationMode::modes(M2_N_MODE, 1e-6).start_from(2),
                    )?;
                calib_sh48_bm.pseudoinverse();
                serde_pickle::to_writer(
                    &mut File::create(format!("calib_sh48_{M1_N_MODE}bm.pkl"))?,
                    &calib_sh48_bm,
                    Default::default(),
                )?;
                calib_sh48_bm
            };
        Ok(calib_sh48_bm)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crseo::{Atmosphere, RayTracing};
    use gmt_dos_clients::gif;
    use gmt_dos_clients_io::optics::{Dev, Frame, Host, SensorData};
    use interface::{Read, Update, Write};
    use std::error::Error;

    #[test]
    fn sh48() -> Result<(), Box<dyn Error>> {
        let models = Models::new();
        let mut sh48 = models.sh48::<10>().build()?;
        println!("{sh48}");
        let mut centroids = models.sh48::<10>().processor()?;
        let mut frame = gif::Frame::<f32>::new("sh48.png", 48 * 8);
        sh48.update();
        <OpticalModel<_> as Write<Frame<Host>>>::write(&mut sh48).map(|data| {
            dbg!(data.len());
            <gif::Frame<_> as Read<Frame<Host>>>::read(&mut frame, data)
        });
        <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut sh48)
            .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut centroids, data));
        centroids.update();
        let data = <CentroidsProcessing as Write<SensorData>>::write(&mut centroids)
            .unwrap()
            .into_arc();
        dbg!((data.len(), 48 * 48 * 3 * 2));
        dbg!(data.iter().sum::<f64>());
        frame.update();
        frame.save()?;
        Ok(())
    }
    #[test]
    fn sh48_with_atmosphere() -> Result<(), Box<dyn Error>> {
        let atm_builder = Atmosphere::builder()
            .single_turbulence_layer(0f32, Some(7f32), Some(0f32))
            .ray_tracing(
                RayTracing::default()
                    .duration(5.)
                    .n_duration(100)
                    .filepath("atm_single_layer.bin"),
            );
        let models = Models::new().atmosphere(500., atm_builder);
        const R: usize = 1000;
        let mut sh48 = models.sh48::<R>().build()?;
        println!("{sh48}");
        println!("init centroids");
        let mut centroids = models.sh48::<R>().processor()?;
        let mut frame = gif::Frame::<f32>::new("sh48_with_atmosphere.png", 48 * 8);
        println!("integrating...");
        for _ in 0..R {
            sh48.update();
        }
        <OpticalModel<_> as Write<Frame<Host>>>::write(&mut sh48).map(|data| {
            dbg!(data.len());
            <gif::Frame<_> as Read<Frame<Host>>>::read(&mut frame, data)
        });
        <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut sh48)
            .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut centroids, data));
        centroids.update();
        let data = <CentroidsProcessing as Write<SensorData>>::write(&mut centroids)
            .unwrap()
            .into_arc();
        dbg!((data.len(), 48 * 48 * 3 * 2));
        dbg!(data.iter().sum::<f64>());
        frame.update();
        frame.save()?;
        Ok(())
    }
}
