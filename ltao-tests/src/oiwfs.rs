use crseo::{gmt::GmtM2, imaging::Detector, FromBuilder, Source};
use gmt_dos_clients_crseo::{
    calibration::{GlobalCalibration, Reconstructor},
    centroiding::CentroidsProcessing,
    sensors::{builders::CameraBuilder, Camera},
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
};
use skyangle::Conversion;
use std::fs::File;

use crate::{Model, Oiwfs};
impl Oiwfs {
    pub fn oiwfs() -> CameraBuilder {
        // OIWFS: imager
        let oiwfs_n_px = 255;
        Camera::builder().detector(Detector::default().n_px_imagelet(oiwfs_n_px))
    }
}
impl Model for Oiwfs {
    type Sensor = Camera;
    type Processor = CentroidsProcessing;
    type Estimator = Reconstructor;
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder> {
        OpticalModel::<Camera<1>>::builder()
            .gmt(self.gmt_builder.clone())
            .source(Source::builder().band("K"))
            .sensor(Oiwfs::oiwfs())
    }
    fn build(&self) -> anyhow::Result<OpticalModel<Self::Sensor>> {
        Ok(self.builder().build()?)
    }
    fn processor(&self) -> anyhow::Result<Self::Processor> {
        // OIWFS: centroids processing
        let mut centroids = CentroidsProcessing::try_from(&Oiwfs::oiwfs())?;
        self.builder().initialize(&mut centroids);
        Ok(centroids)
    }
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator> {
        // println!(" -- OIWFS CALIBRATION -- ");
        // OIWFS: global tip-tilt calibration
        let calib_oiwfs: Reconstructor = if let Ok(file) = File::open("calib_oiwfs.pkl") {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let mut calib_oiwfs = <CentroidsProcessing as GlobalCalibration<GmtM2>>::calibrate(
                &(&self.builder()).into(),
                gmt_dos_clients_crseo::calibration::CalibrationMode::GlobalTipTilt(
                    100f64.from_mas(),
                ),
            )?;
            calib_oiwfs.pseudoinverse();
            serde_pickle::to_writer(
                &mut File::create("calib_oiwfs.pkl")?,
                &calib_oiwfs,
                Default::default(),
            )?;
            calib_oiwfs
        };
        // println!("{calib_oiwfs}");
        Ok(calib_oiwfs)
    }
}
