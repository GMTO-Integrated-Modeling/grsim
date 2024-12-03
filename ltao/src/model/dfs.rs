use std::{
    fs::File,
    marker::PhantomData,
    ops::{Deref, DerefMut},
    path::Path,
};

use crseo::FromBuilder;
use gmt_dos_clients_crseo::{
    calibration::{
        algebra::{Block, Collapse},
        CalibrationMode, MirrorMode, MixedMirrorMode, Reconstructor,
    },
    sensors::DispersedFringeSensor,
    DeviceInitialize, DispersedFringeSensorProcessing, OpticalModel, OpticalModelBuilder,
};
use gmt_dos_clients_io::optics::{
    dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
    Dev,
};

use crate::{kernels::KernelSpecs, M1RbmM2modes, Model, Models};

pub enum Rxy {}
pub enum RxyPiston {}
pub trait DfsModes {}
impl DfsModes for Rxy {}
impl DfsModes for RxyPiston {}
impl DfsModes for () {}

pub struct Dfs<M: DfsModes, const C: usize, const F: usize>(
    pub(crate) Models,
    pub(crate) PhantomData<M>,
);
impl<M: DfsModes, const C: usize, const F: usize> Deref for Dfs<M, C, F> {
    type Target = Models;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<M: DfsModes, const C: usize, const F: usize> DerefMut for Dfs<M, C, F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl<M: DfsModes, const F: usize, const C: usize> KernelSpecs for Dfs<M, C, F> {
    type Integrator = gmt_dos_clients::Integrator<M1RbmM2modes>;
    type Input = DfsFftFrame<Dev>;
    type Data = Intercepts;
    type Output = M1RbmM2modes;
}
impl<const C: usize, const F: usize> Model for Dfs<RxyPiston, C, F> {
    type Sensor = DispersedFringeSensor<C, F>;
    type Processor = DispersedFringeSensorProcessing;
    type Estimator = Reconstructor<MixedMirrorMode>;
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder> {
        OpticalModel::<DispersedFringeSensor<C, F>>::builder()
            .gmt(self.gmt_builder.clone())
            .source(self.agws_gss.clone())
            .sensor(DispersedFringeSensor::<C, F>::builder().source(self.agws_gss.clone()))
    }
    // fn build(&self) -> anyhow::Result<OpticalModel<Self::Sensor>> {
    //     Ok(self.builder().build()?)
    // }
    fn processor(&self) -> anyhow::Result<Self::Processor> {
        let mut processor = DispersedFringeSensorProcessing::new();
        self.builder().initialize(&mut processor);
        Ok(processor)
    }
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator> {
        let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples")
            .join("dfs_closed-loop_calibration");
        // .join("calib_ltws-oiwfs_dfs_m1-rxy.pkl");
        // println!(" -- DFS CALIBRATION -- ");
        // Ok(serde_pickle::from_reader(
        //     &mut File::open(data_path)?,
        //     Default::default(),
        // )?)
        let recon_rxy: Reconstructor<MirrorMode> = serde_pickle::from_reader(
            &mut File::open(data_path.join("calib_ltws-oiwfs_dfs_m1-rxy.pkl"))?,
            Default::default(),
        )?;
        let recon_tz: Reconstructor<CalibrationMode> = serde_pickle::from_reader(
            &mut File::open(data_path.join("calib_dfs_m2-piston.pkl"))?,
            Default::default(),
        )?;
        let recon_tz = recon_tz.collapse();
        // let mut split_recon_rxy = recon_rxy.split();
        // split_recon_rxy.merge(recon_tz);
        // let mut recon = split_recon_rxy.collapse();
        let mut recon = Block::block(&[&[&recon_rxy, &recon_tz]]);
        recon.truncated_pseudoinverse(vec![2]);
        Ok(recon)
        /* let recon_rxy = if let Ok(file) = File::open(data_path)) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let closed_loop_optical_model =
                OpticalModel::<WaveSensor>::builder().gmt(self.gmt_builder.clone());
            let recon_rxy = <DispersedFringeSensorProcessing as ClosedLoopCalibration<
                WaveSensor,
            >>::calibrate_serial(
                &self.builder().clone_into::<1, 1>(),
                MirrorMode::from(CalibrationMode::RBM([
                    None,                    // Tx
                    None,                    // Ty
                    None,                    // Tz
                    Some(100f64.from_mas()), // Rx
                    Some(100f64.from_mas()), // Ry
                    None,                    // Rz
                ]))
                .update((7, CalibrationMode::empty_rbm())),
                &closed_loop_optical_model,
                CalibrationMode::modes(M2_N_MODE, 1e-6),
            )?;
            let mut recon_rxy = recon_rxy.collapse();
            recon_rxy.pseudoinverse();
            serde_pickle::to_writer(
                &mut File::create("calib_dfs_m1-rxy.pkl")?,
                &recon_rxy,
                Default::default(),
            )?;
            recon_rxy
        };
        // println!("{recon_rxy}");
        Ok(recon_rxy)*/
    }
}
impl<const C: usize, const F: usize> Model for Dfs<Rxy, C, F> {
    type Sensor = DispersedFringeSensor<C, F>;
    type Processor = DispersedFringeSensorProcessing;
    type Estimator = Reconstructor<MirrorMode>;
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder> {
        OpticalModel::<DispersedFringeSensor<C, F>>::builder()
            .gmt(self.gmt_builder.clone())
            .source(self.agws_gss.clone())
            .sensor(DispersedFringeSensor::<C, F>::builder().source(self.agws_gss.clone()))
    }
    // fn build(&self) -> anyhow::Result<OpticalModel<Self::Sensor>> {
    //     Ok(self.builder().build()?)
    // }
    fn processor(&self) -> anyhow::Result<Self::Processor> {
        let mut processor = DispersedFringeSensorProcessing::new();
        self.builder().initialize(&mut processor);
        Ok(processor)
    }
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator> {
        let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples")
            .join("dfs_closed-loop_calibration");
        // .join("calib_ltws-oiwfs_dfs_m1-rxy.pkl");
        // println!(" -- DFS CALIBRATION -- ");
        // Ok(serde_pickle::from_reader(
        //     &mut File::open(data_path)?,
        //     Default::default(),
        // )?)
        let recon_rxy: Reconstructor<MirrorMode> = serde_pickle::from_reader(
            &mut File::open(data_path.join("calib_ltws-oiwfs_dfs_m1-rxy.pkl"))?,
            Default::default(),
        )?;
        /* let recon_tz: Reconstructor<CalibrationMode> = serde_pickle::from_reader(
            &mut File::open(data_path.join("calib_dfs_m1-tz.pkl"))?,
            Default::default(),
        )?;
        let recon_tz = recon_tz.collapse();
        // let mut split_recon_rxy = recon_rxy.split();
        // split_recon_rxy.merge(recon_tz);
        // let mut recon = split_recon_rxy.collapse();
        let mut recon = Block::block(&[&[&recon_rxy, &recon_tz]]);
        recon.pseudoinverse(); */
        Ok(recon_rxy)
        /* let recon_rxy = if let Ok(file) = File::open(data_path)) {
            serde_pickle::from_reader(file, Default::default())?
        } else {
            let closed_loop_optical_model =
                OpticalModel::<WaveSensor>::builder().gmt(self.gmt_builder.clone());
            let recon_rxy = <DispersedFringeSensorProcessing as ClosedLoopCalibration<
                WaveSensor,
            >>::calibrate_serial(
                &self.builder().clone_into::<1, 1>(),
                MirrorMode::from(CalibrationMode::RBM([
                    None,                    // Tx
                    None,                    // Ty
                    None,                    // Tz
                    Some(100f64.from_mas()), // Rx
                    Some(100f64.from_mas()), // Ry
                    None,                    // Rz
                ]))
                .update((7, CalibrationMode::empty_rbm())),
                &closed_loop_optical_model,
                CalibrationMode::modes(M2_N_MODE, 1e-6),
            )?;
            let mut recon_rxy = recon_rxy.collapse();
            recon_rxy.pseudoinverse();
            serde_pickle::to_writer(
                &mut File::create("calib_dfs_m1-rxy.pkl")?,
                &recon_rxy,
                Default::default(),
            )?;
            recon_rxy
        };
        // println!("{recon_rxy}");
        Ok(recon_rxy)*/
    }
}
impl<const C: usize, const F: usize> Model for Dfs<(), C, F> {
    type Sensor = DispersedFringeSensor<C, F>;
    type Processor = DispersedFringeSensorProcessing;
    type Estimator = Reconstructor<MirrorMode>;
    fn builder(&self) -> OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder> {
        OpticalModel::<DispersedFringeSensor<C, F>>::builder()
            .gmt(self.gmt_builder.clone())
            .source(self.agws_gss.clone())
            .sensor(DispersedFringeSensor::<C, F>::builder().source(self.agws_gss.clone()))
    }
    // fn build(&self) -> anyhow::Result<OpticalModel<Self::Sensor>> {
    //     Ok(self.builder().build()?)
    // }
    fn processor(&self) -> anyhow::Result<Self::Processor> {
        let mut processor = DispersedFringeSensorProcessing::new();
        self.builder().initialize(&mut processor);
        Ok(processor)
    }
    fn reconstructor(&self) -> anyhow::Result<Self::Estimator> {
        unimplemented!()
    }
}
#[cfg(test)]
mod tests {
    use crate::Models;

    use super::*;
    use crseo::{Atmosphere, RayTracing};
    use gmt_dos_clients::gif;
    use gmt_dos_clients_io::optics::{dispersed_fringe_sensor::DfsFftFrame, Frame, Host};
    use interface::{Read, Update, Write};
    use std::error::Error;

    #[test]
    fn dfs() -> Result<(), Box<dyn Error>> {
        let models = Models::new();
        let mut dfs = models.dfs::<(), 1, 1>().build()?;
        println!("{dfs}");
        let mut frame = gif::Frame::<f32>::new("dfs.png", 258);
        dfs.update();
        <OpticalModel<_> as Write<Frame<Host>>>::write(&mut dfs).map(|data| {
            dbg!(data.len());
            <gif::Frame<_> as Read<Frame<Host>>>::read(&mut frame, data)
        });
        frame.update();
        frame.save()?;
        Ok(())
    }
    #[test]
    fn dfs_fft() -> Result<(), Box<dyn Error>> {
        let models = Models::new();
        let mut dfs = models.dfs::<(), 1, 1>().build()?;
        println!("{dfs}");
        let mut frame = gif::Frame::<f32>::new("dfs_fft.png", 516);
        dfs.update();
        <OpticalModel<_> as Write<DfsFftFrame<Host>>>::write(&mut dfs).map(|data| {
            dbg!(data.len());
            <gif::Frame<_> as Read<DfsFftFrame<Host>>>::read(&mut frame, data)
        });
        frame.update();
        frame.save()?;
        Ok(())
    }
    #[test]
    fn dfs_with_atmosphere() -> Result<(), Box<dyn Error>> {
        let atm_builder = Atmosphere::builder()
            .single_turbulence_layer(0f32, Some(7f32), Some(0f32))
            .ray_tracing(
                RayTracing::default()
                    .duration(5.)
                    .n_duration(100)
                    .filepath("atm_single_layer.bin"),
            );
        let models = Models::new().atmosphere(500., atm_builder);
        const R: usize = 5;
        let mut dfs = models.dfs::<(), R, R>().build()?;
        println!("{dfs}");
        let mut frame = gif::Frame::<f32>::new("dfs_with_atmosphere.png", 258);
        for _ in 0..R {
            dfs.update();
        }
        <OpticalModel<_> as Write<Frame<Host>>>::write(&mut dfs).map(|data| {
            // dbg!(data.len());
            <gif::Frame<_> as Read<Frame<Host>>>::read(&mut frame, data)
        });
        frame.update();
        frame.save()?;
        Ok(())
    }
    #[test]
    fn dfs_fft_with_atmosphere() -> Result<(), Box<dyn Error>> {
        let atm_builder = Atmosphere::builder()
            .single_turbulence_layer(0f32, Some(7f32), Some(0f32))
            .ray_tracing(
                RayTracing::default()
                    .duration(5.)
                    .n_duration(100)
                    .filepath("atm_single_layer.bin"),
            );
        let models = Models::new().atmosphere(500., atm_builder);
        const R: usize = 5;
        const F: usize = 1000;
        let mut dfs = models.dfs::<(), R, F>().build()?;
        println!("{dfs}");
        let mut frame = gif::Frame::<f32>::new("dfs_fft_with_atmosphere.png", 516);
        for _ in 0..F * R {
            dfs.update();
        }
        <OpticalModel<_> as Write<DfsFftFrame<Host>>>::write(&mut dfs).map(|data| {
            dbg!(data.len());
            <gif::Frame<_> as Read<DfsFftFrame<Host>>>::read(&mut frame, data)
        });
        frame.update();
        frame.save()?;
        Ok(())
    }
}
