use crate::{M1_N_MODE, M2_N_MODE, N_ACTUATOR};

use gmt_dos_clients_io::{
    gmt_m1::{assembly::M1ActuatorCommandForces, M1ModeShapes},
    gmt_m2::asm::M2ASMAsmCommand,
    optics::M2modes,
};
use interface::{Data, Read, Update, Write};
use matio_rs::{MatFile, MatioError};
use nalgebra as na;
use std::{env, error::Error, mem, path::Path, sync::Arc};

#[derive(Debug)]
pub struct ModalToZonal {
    mats: Vec<na::DMatrix<f64>>,
    modes: Arc<Vec<f64>>,
    actuators: Vec<f64>,
    n_mode: usize,
}

impl ModalToZonal {
    pub fn new(
        mat_file: &str,
        mat_var_prefix: &str,
        n_mode: usize,
    ) -> Result<Self, Box<dyn Error>> {
        let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set!");
        let fem_path = Path::new(&fem_var);
        let mat_file = MatFile::load(&fem_path.join(mat_file))?;
        Ok(Self {
            mats: (1..=7)
                .map(|i| mat_file.var::<_, na::DMatrix<f64>>(format!("{}_{}", mat_var_prefix, i)))
                .collect::<Result<Vec<na::DMatrix<f64>>, MatioError>>()?,
            modes: Arc::new(vec![0.0; M2_N_MODE]),
            actuators: vec![0.0; N_ACTUATOR],
            n_mode,
        })
    }
    pub fn asms() -> Result<Self, Box<dyn Error>> {
        Self::new("asms_eigenmodes_wz123.mat", "KL", M2_N_MODE)
    }
    pub fn m1() -> Result<Self, Box<dyn Error>> {
        Self::new("m1_bm2force.mat", "b2f", M1_N_MODE)
    }
}

impl Update for ModalToZonal {
    fn update(&mut self) {
        // self.mats.iter().for_each(|m| println!("{:?}", m.shape()));
        // dbg!(self.modes.len());
        let _ = mem::replace(
            &mut self.actuators,
            self.modes
                .chunks(self.n_mode)
                .zip(self.mats.iter())
                .map(|(modes, mat)| mat * na::DVector::from_column_slice(modes))
                .flat_map(|actuators| actuators.as_slice().to_vec())
                .collect(),
        );
    }
}

impl Read<M2modes> for ModalToZonal {
    fn read(&mut self, data: Data<M2modes>) {
        self.modes = data.into_arc();
    }
}
impl Read<M1ModeShapes> for ModalToZonal {
    fn read(&mut self, data: Data<M1ModeShapes>) {
        self.modes = data.into_arc();
    }
}

impl Write<M2ASMAsmCommand> for ModalToZonal {
    fn write(&mut self) -> Option<Data<M2ASMAsmCommand>> {
        Some(self.actuators.clone().into())
    }
}

impl Write<M1ActuatorCommandForces> for ModalToZonal {
    fn write(&mut self) -> Option<Data<M1ActuatorCommandForces>> {
        Some(self.actuators.clone().into())
    }
}
