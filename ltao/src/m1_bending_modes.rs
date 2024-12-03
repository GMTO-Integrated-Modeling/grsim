use std::{fs::File, path::Path, sync::Arc};

use gmt_dos_clients_io::{
    gmt_fem::outputs::M1Segment1AxialD,
    gmt_m1::{
        assembly::M1ActuatorCommandForces,
        segment::{ActuatorCommandForces, BendingModes},
    },
};
use interface::{Data, Read, Update, Write};
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct SingularModes {
    mode_nodes: Vec<Vec<f64>>,
    actuator_nodes: Vec<Vec<f64>>,
    modes: Vec<f64>,
    mode_2_force: Vec<f64>,
    shape: (usize, usize),
}

#[derive(Debug, Default, Clone)]
pub struct M1BendingModes {
    modes: SingularModes,
    mode_coefficients: Arc<Vec<f64>>,
    n_mode: usize,
    axial_d: Arc<Vec<f64>>,
}

impl M1BendingModes {
    pub fn new<P: AsRef<Path>>(path: P, n_mode: usize) -> anyhow::Result<Self> {
        let modes: SingularModes =
            serde_pickle::from_reader(&mut File::open(path.as_ref())?, Default::default())?;
        Ok(Self {
            modes,
            n_mode,
            ..Default::default()
        })
    }
}
impl Update for M1BendingModes {
    fn update(&mut self) {}
}
impl<const ID: u8> Read<BendingModes<ID>> for M1BendingModes {
    fn read(&mut self, data: Data<BendingModes<ID>>) {
        self.mode_coefficients = data.into_arc();
    }
}
impl<const ID: u8> Write<ActuatorCommandForces<ID>> for M1BendingModes {
    fn write(&mut self) -> Option<Data<ActuatorCommandForces<ID>>> {
        let (_ns, na) = self.modes.shape;
        let m2f = faer::mat::from_column_major_slice::<f64>(&self.modes.mode_2_force, na, na);
        let forces = m2f.subcols(0, self.n_mode)
            * faer::mat::from_column_major_slice::<f64>(&self.mode_coefficients, self.n_mode, 1);
        Some(forces.col_as_slice(0).into())
    }
}
// impl Write<M1ActuatorCommandForces> for M1BendingModes {
// fn write(&mut self) -> Option<Data<M1ActuatorCommandForces>> {
// Some(forces.into())
// }
// }
impl Read<M1Segment1AxialD> for M1BendingModes {
    fn read(&mut self, data: Data<M1Segment1AxialD>) {
        self.axial_d = data.into_arc();
    }
}
impl<const ID: u8> Write<BendingModes<ID>> for M1BendingModes {
    fn write(&mut self) -> Option<Data<BendingModes<ID>>> {
        let (ns, na) = self.modes.shape;
        let surface = faer::mat::from_column_major_slice::<f64>(&self.axial_d, ns, 1);
        let u = faer::mat::from_column_major_slice::<f64>(&self.modes.modes, ns, na);
        let c = u.subcols(0, self.n_mode).transpose() * surface;
        Some(c.col_as_slice(0).into())
    }
}
