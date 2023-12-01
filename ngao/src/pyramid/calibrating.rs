use std::fmt::Display;
use std::ops::Mul;
use std::time::Instant;

use crseo::wavefrontsensor::PyramidBuilder;

use interface::UniqueIdentifier;
use nalgebra as na;
use serde::{Deserialize, Serialize};

use crate::pyramid::PyramidData;
use crate::{Calibrating, CalibratingError, Calibration};

mod builder;
pub use builder::PyramidCalibratorBuilder;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Segment {
    sid: u8,
    n_mode: usize,
    mask: na::DMatrix<bool>,
    calibration: na::DMatrix<f32>,
}

impl Display for Segment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Segment #{} with {} modes, calibration: {:?}, mask: {:?}",
            self.sid,
            self.n_mode,
            self.calibration.shape(),
            self.mask.shape(),
        )
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PyramidCalibrator {
    n_mode: usize,
    segments: Vec<Segment>,
    piston_mask: (Vec<bool>, Vec<bool>),
    h_filter: Vec<bool>,
    p_filter: Vec<bool>,
    offset: Vec<f32>,
    h_matrix: na::DMatrix<f32>,
    p_matrix: na::DMatrix<f32>,
    estimator: Option<Estimator>,
}

impl Display for PyramidCalibrator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let n = self.segments.len();
        let px = self
            .piston_mask
            .0
            .iter()
            .filter(|&&f| f)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        let py = self
            .piston_mask
            .0
            .iter()
            .filter(|&&f| f)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        let hn = self
            .h_filter
            .iter()
            .filter(|&&f| f)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        let pn = self
            .p_filter
            .iter()
            .filter(|&&f| f)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        writeln!(
            f,
            r"
Pyramid calibrator for {} segments with {} modes:
 * masks:
   * piston non-zeros: [{},{}]
   * H filter non zeros: {}
   * P filter non zeros: {}
   * H matrix: {:?}
   * P matrix: {:?}
 * offset vector size: {}
 * {}
        ",
            n,
            self.n_mode,
            px,
            py,
            hn,
            pn,
            self.h_matrix.shape(),
            self.p_matrix.shape(),
            self.offset.len(),
            self.estimator
                .as_ref()
                .map_or("no estimator".to_string(), |estimator| estimator
                    .to_string())
        )?;
        for segment in &self.segments {
            writeln!(f, "{segment}")?;
        }
        Ok(())
    }
}

impl Calibrating for PyramidCalibrator {
    type ProcessorData = PyramidData<f32>;
    type Output = Vec<f64>;
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum Estimator {
    H0(na::DMatrix<f32>),
    H(na::DMatrix<f32>),
    P(na::DMatrix<f32>),
    HP(na::DMatrix<f32>),
    ConstrainedHP(na::DMatrix<f32>),
}

impl Display for Estimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Estimator::H0(mat) => write!(f, "H0 estimator size: {:?}", mat.shape()),
            Estimator::H(mat) => write!(f, "H estimator size: {:?}", mat.shape()),
            Estimator::P(mat) => write!(f, "P estimator size: {:?}", mat.shape()),
            Estimator::HP(mat) => write!(f, "HP estimator size: {:?}", mat.shape()),
            Estimator::ConstrainedHP(mat) => {
                write!(f, "constrained HP estimator size: {:?}", mat.shape())
            }
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum PyramidCalibratorError {
    #[error("failed to compute H matrix pseudo-inverse: {0}")]
    HEstimator(String),
    #[error("failed to compute P matrix pseudo-inverse: {0}")]
    PEstimator(String),
    #[error("failed to compute HP matrix inverse")]
    HPEstimator,
}

impl PyramidCalibrator {
    pub fn builder(
        pym: PyramidBuilder,
        modes: impl Into<String>,
        n_mode: usize,
    ) -> PyramidCalibratorBuilder {
        PyramidCalibratorBuilder {
            pym,
            sids: vec![1, 2, 3, 4, 5, 6, 7],
            modes: modes.into(),
            n_mode,
            n_gpu: 1,
            n_thread: None,
            piston_mask_threshold: 0.55,
        }
    }
    pub fn data(&self, rhs: &PyramidData<f32>) -> Vec<f32> {
        let PyramidData { sx, sy, .. } = rhs;
        sx.iter()
            .chain(sy.iter())
            .zip(self.h_filter.iter().cycle())
            .filter_map(|(s, h)| h.then_some(*s))
            .chain(
                sx.iter()
                    .chain(sy.iter())
                    .zip(&self.p_filter)
                    .filter_map(|(s, p)| p.then_some(*s)),
            )
            .collect()
    }
    #[cfg(feature = "faer")]
    pub fn p_matrix_cond(&self) -> f32 {
        use faer::{solvers::Svd, IntoFaer};
        let mat = self.p_matrix.view_range(.., ..).into_faer();
        let svd = Svd::new(mat);
        let s_diag = svd.s_diagonal();
        let k = s_diag.nrows() - 1;
        s_diag[(0, 0)] / s_diag[(k, 0)]
    }
    #[cfg(feature = "faer")]
    pub fn h_matrix_cond(&self) -> f32 {
        use faer::{solvers::Svd, IntoFaer};
        let mat = self.h_matrix.view_range(.., ..).into_faer();
        let svd = Svd::new(mat);
        let s_diag = svd.s_diagonal();
        let k = s_diag.nrows() - 1;
        s_diag[(0, 0)] / s_diag[(k, 0)]
    }
    #[cfg(not(feature = "faer"))]
    pub fn h0_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        println!("computing the H0 estimator (with nalgebra) ...");
        let now = Instant::now();
        let indices: Vec<_> = (0..6).map(|i| i * self.n_mode).collect();
        let h0_matrix = self.h_matrix.clone().remove_columns_at(&indices);
        let mat = h0_matrix
            .pseudo_inverse(0f32)
            .map_err(|e| PyramidCalibratorError::HEstimator(e.into()))?;
        self.estimator = Some(Estimator::H0(mat));
        println!("  ... in {}s", now.elapsed().as_secs());
        Ok(self)
    }
    #[cfg(feature = "faer")]
    pub fn h0_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        use faer::{solvers::Svd, IntoFaer, IntoNalgebra, Mat};

        println!("computing the H0 estimator (with faer) ...");
        let now = Instant::now();
        let indices: Vec<_> = (0..6).map(|i| i * self.n_mode).collect();
        let h0_matrix = self.h_matrix.clone().remove_columns_at(&indices);
        let h0_matrix = h0_matrix.view_range(.., ..).into_faer();

        let m = h0_matrix.nrows();
        let n = h0_matrix.ncols();
        let svd = Svd::new(h0_matrix);

        let s_diag = svd.s_diagonal();
        let mut s_inv = Mat::zeros(n, m);
        for i in 0..Ord::min(m, n) {
            s_inv[(i, i)] = 1.0 / s_diag[(i, 0)];
        }

        let mat = svd.v() * &s_inv * svd.u().adjoint();

        self.estimator = Some(Estimator::H0(mat.as_ref().into_nalgebra().into()));
        println!("  ... in {}s", now.elapsed().as_secs());
        Ok(self)
    }
    pub fn h_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        let mat = self
            .h_matrix
            .clone()
            .pseudo_inverse(0f32)
            .map_err(|e| PyramidCalibratorError::HEstimator(e.into()))?;
        self.estimator = Some(Estimator::H(mat));
        Ok(self)
    }
    pub fn p_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        let mat = self
            .p_matrix
            .clone()
            .pseudo_inverse(0f32)
            .map_err(|e| PyramidCalibratorError::PEstimator(e.into()))?;
        self.estimator = Some(Estimator::P(mat));
        Ok(self)
    }
    pub fn hp_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        println!("computing the pyramid LSQ reconstructor ...");
        let now = Instant::now();
        let hp_matrix =
            self.h_matrix.transpose() * &self.h_matrix + self.p_matrix.transpose() * &self.p_matrix;
        let columns: Vec<_> = self
            .h_matrix
            .transpose()
            .column_iter()
            .map(|column| column.clone_owned())
            .chain(
                self.p_matrix
                    .transpose()
                    .column_iter()
                    .map(|column| column.clone_owned()),
            )
            .collect();
        let m_matrix = na::DMatrix::from_columns(&columns);
        let reconstructor = hp_matrix
            .try_inverse()
            .ok_or(PyramidCalibratorError::HPEstimator)?
            * &m_matrix;
        println!(
            "Reconstructor: [{:?}] in {}s",
            reconstructor.shape(),
            now.elapsed().as_secs()
        );
        self.estimator = Some(Estimator::HP(reconstructor));
        Ok(self)
    }
    pub fn set_hp_estimator(&mut self, reconstructor: na::DMatrix<f32>) -> &mut Self {
        self.estimator = Some(Estimator::HP(reconstructor));
        self
    }
}

impl From<PyramidCalibrator> for Calibration<PyramidCalibrator> {
    fn from(calibrator: PyramidCalibrator) -> Self {
        Self {
            calibrator,
            output: Default::default(),
        }
    }
}

pub enum PyramidCommand {}
impl UniqueIdentifier for PyramidCommand {
    type DataType = Vec<f64>;
}

impl Mul<&PyramidData<f32>> for &PyramidCalibrator {
    type Output = <PyramidCalibrator as Calibrating>::Output;

    fn mul(self, rhs: &PyramidData<f32>) -> Self::Output {
        let sxy: Vec<_> = self
            .data(rhs)
            .into_iter()
            .zip(&self.offset)
            .map(|(s, s0)| s - *s0)
            .collect();
        self.estimator
            .as_ref()
            .map(|estimator| {
                match estimator {
                    Estimator::H0(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy[..self.h_matrix.nrows()]);
                        let mut v = v.as_slice().to_vec();
                        for i in 0..7 {
                            v.insert(i * self.n_mode, 0f32);
                        }
                        v
                    }
                    Estimator::H(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy[..self.h_matrix.nrows()]);
                        v.as_slice().to_vec()
                    }
                    Estimator::P(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy[self.h_matrix.nrows()..]);
                        let mut v = v.as_slice().to_vec();
                        v.insert(6 * self.n_mode, 0f32);
                        v
                    }
                    Estimator::HP(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy);
                        v.as_slice().to_vec()
                    }
                    Estimator::ConstrainedHP(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy);
                        v.as_slice().to_vec()
                    }
                }
                .into_iter()
                .map(|x| x as f64)
                .collect::<Vec<_>>()
            })
            .unwrap_or_default()
    }
}
