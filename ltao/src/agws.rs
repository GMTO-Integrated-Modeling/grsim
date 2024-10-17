use crseo::{
    gmt::GmtBuilder,
    imaging::{Detector, LensletArray},
    source::SourceBuilder,
    FromBuilder, Gmt, Source,
};
use gmt_dos_clients_crseo::{
    sensors::{
        builders::{CameraBuilder, DispersedFringeSensorBuilder},
        Camera, DispersedFringeSensor,
    },
    OpticalModel, OpticalModelBuilder,
};
use skyangle::Conversion;

#[derive(Default, Debug, Clone)]
pub struct Agws<
    const M1_N_MODE: usize = 27,
    const M2_N_MODE: usize = 66,
    const AGWS_N_GS: usize = 3,
    const SH48_CAMERA_EXPOSURE: usize = 1,
    const DFS_CAMERA_EXPOSURE: usize = 1,
    const DFS_FFT_EXPOSURE: usize = 1,
> {
    sampling_frequency: f64,
    gmt: GmtBuilder,
    source: SourceBuilder,
    dfs: DispersedFringeSensorBuilder<DFS_CAMERA_EXPOSURE, DFS_FFT_EXPOSURE>,
    sh48: CameraBuilder<SH48_CAMERA_EXPOSURE>,
}
impl<
        const M1_N_MODE: usize,
        const M2_N_MODE: usize,
        const AGWS_N_GS: usize,
        const SH48_CAMERA_EXPOSURE: usize,
        const DFS_CAMERA_EXPOSURE: usize,
        const DFS_FFT_EXPOSURE: usize,
    >
    Agws<
        M1_N_MODE,
        M2_N_MODE,
        AGWS_N_GS,
        SH48_CAMERA_EXPOSURE,
        DFS_CAMERA_EXPOSURE,
        DFS_FFT_EXPOSURE,
    >
{
    /// Creates a new AGWS model
    pub fn new(sampling_frequency: f64) -> Self {
        Self {
            sampling_frequency,
            gmt: Gmt::builder()
                .m1("bending modes", M1_N_MODE)
                .m2("Karhunen-Loeve", M2_N_MODE)
                .m1_truss_projection(false),
            source: Source::builder()
                .size(AGWS_N_GS)
                .on_ring(6f32.from_arcmin()),
            dfs: DispersedFringeSensor::builder(),
            sh48: Camera::builder()
                .lenslet_array(
                    LensletArray::default()
                        .n_side_lenslet(48)
                        .n_px_lenslet(24)
                        .pitch(0.53),
                )
                .detector(Detector::default().n_px_framelet(8))
                .lenslet_flux(0.75),
            ..Default::default()
        }
    }
    /// Sets the guide stars properties
    pub fn source(mut self, source: SourceBuilder) -> Self {
        self.source = source;
        self
    }
    /// Sets the GMT properies
    pub fn gmt(mut self, gmt: GmtBuilder) -> Self {
        self.gmt = gmt;
        self
    }
    /// Returns the DFS builder
    pub fn dfs(
        &self,
    ) -> OpticalModelBuilder<DispersedFringeSensorBuilder<DFS_CAMERA_EXPOSURE, DFS_FFT_EXPOSURE>>
    {
        OpticalModel::<DispersedFringeSensor<DFS_CAMERA_EXPOSURE, DFS_FFT_EXPOSURE>>::builder()
            .sampling_frequency(self.sampling_frequency)
            .gmt(self.gmt.clone())
            .source(self.source.clone().band("J"))
            // .atmosphere(atm_builder)
            .sensor(self.dfs.clone().source(self.source.clone().band("J")))
    }
    /// Returns the SH48 builder
    pub fn sh48(&self) -> OpticalModelBuilder<CameraBuilder<SH48_CAMERA_EXPOSURE>> {
        OpticalModel::<Camera<SH48_CAMERA_EXPOSURE>>::builder()
            .gmt(self.gmt.clone())
            .source(
                self.source
                    .clone()
                    .pupil_size(48 as f64 * 0.53)
                    .band("R")
                    .fwhm(6.),
            )
            .sensor(self.sh48.clone())
    }
}
