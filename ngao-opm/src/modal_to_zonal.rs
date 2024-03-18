use crate::{N_ACTUATOR, N_MODE};

use gmt_dos_clients_io::{gmt_m2::asm::M2ASMAsmCommand, optics::M2modes};
use interface::{Data, Read, Update, Write};
use matio_rs::{MatFile, MatioError};
use nalgebra as na;
use std::{env, error::Error, mem, path::Path, sync::Arc};

#[derive(Debug)]
pub struct ModalToZonal {
    mats: Vec<na::DMatrix<f64>>,
    modes: Arc<Vec<f64>>,
    actuators: Vec<f64>,
}

impl ModalToZonal {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set!");
        let fem_path = Path::new(&fem_var);
        let mat_file = MatFile::load(&fem_path.join("KLmodesGS36p90.mat"))?;
        Ok(Self {
            mats: (1..=7)
                .map(|i| mat_file.var::<_, na::DMatrix<f64>>(format!("KL_{i}")))
                .collect::<Result<Vec<na::DMatrix<f64>>, MatioError>>()?,
            modes: Arc::new(vec![0.0; N_MODE]),
            actuators: vec![0.0; N_ACTUATOR],
        })
    }
}

impl Update for ModalToZonal {
    fn update(&mut self) {
        let _ = mem::replace(
            &mut self.actuators,
            self.modes
                .chunks(N_MODE)
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

impl Write<M2ASMAsmCommand> for ModalToZonal {
    fn write(&mut self) -> Option<Data<M2ASMAsmCommand>> {
        Some(self.actuators.clone().into())
    }
}
