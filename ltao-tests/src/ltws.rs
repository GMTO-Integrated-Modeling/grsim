use gmt_dos_clients_crseo::DeviceInitialize;
use std::fs::File;

use crseo::{gmt::GmtM2, imaging::LensletArray, FromBuilder, Source};
use gmt_dos_clients_crseo::{
    calibration::{Calibration, CalibrationMode, Reconstructor},
    centroiding::{CentroidsProcessing, Full, ZeroMean},
    sensors::Camera,
    OpticalModel, OpticalModelBuilder,
};

use crate::{Ltws, Model, M2_N_MODE};

impl Model for Ltws {
    type Sensor = Camera;
    type Processor = CentroidsProcessing<ZeroMean>;
    type Estimator = Reconstructor;
    fn processor(&self) -> anyhow::Result<Self::Processor> {
        // LTWS: wavefront sensor
        let ltws = Camera::<1>::builder()
            .lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(32))
            // .detector(Detector::default().n_px_framelet(10))
            .lenslet_flux(0.75);
        // LTWS: centroids processing
        let mut centroids = CentroidsProcessing::<ZeroMean>::try_from(&ltws)?;
        self.builder().initialize(&mut centroids);
        Ok(centroids)
    }
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder> {
        // LTWS: wavefront sensor
        let ltws = Camera::builder()
            .lenslet_array(LensletArray::default().n_side_lenslet(60).n_px_lenslet(32))
            // .detector(Detector::default().n_px_framelet(10))
            .lenslet_flux(0.75);
        // LTWS: optical model
        OpticalModel::<Camera<1>>::builder()
            .gmt(self.gmt_builder.clone())
            .source(Source::builder().band("V"))
            .sensor(ltws)
    }
    fn build(&self) -> anyhow::Result<OpticalModel<Self::Sensor>> {
        // LTWS: optical model
        Ok(self.builder().build()?)
    }
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator> {
        // println!(" -- LTWS CALIBRATION -- ");
        let calib_file_name = format!("calib_ltws-{}_m2_modes.pkl", "full");
        let calib_m2_modes: Reconstructor = if let Ok(file) = File::open(&calib_file_name) {
            // println!("loading {calib_file_name}");
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let mut calib_m2_modes = <CentroidsProcessing<Full> as Calibration<GmtM2>>::calibrate(
                &((&self.builder()).into()),
                CalibrationMode::modes(M2_N_MODE, 1e-7).start_from(2),
            )?;
            println!("{} cross-talks", calib_m2_modes.n_cross_talks());
            calib_m2_modes.pseudoinverse();
            serde_pickle::to_writer(
                &mut File::create(&calib_file_name)?,
                &calib_m2_modes,
                Default::default(),
            )?;
            calib_m2_modes
        };
        // println!("{calib_m2_modes}");
        Ok(calib_m2_modes)
    }
}
#[cfg(test)]
mod tests {
    use crate::Models;

    use super::*;
    use gmt_dos_clients::gif;
    use gmt_dos_clients_io::optics::{Frame, Host};
    use interface::{Read, Update, Write};
    use std::error::Error;

    #[test]
    fn ltws() -> Result<(), Box<dyn Error>> {
        let models = Models::new();
        let mut ltws = models.ltws().build()?;
        println!("{ltws}");
        let mut frame = gif::Frame::<f32>::new("ltws.png", 60 * 32);
        ltws.update();
        <OpticalModel<_> as Write<Frame<Host>>>::write(&mut ltws).map(|data| {
            dbg!(data.len());
            <gif::Frame<_> as Read<Frame<Host>>>::read(&mut frame, data)
        });
        frame.update();
        frame.save()?;
        Ok(())
    }
}
