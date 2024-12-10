use std::{env, fs::File, path::Path, sync::Arc};

use gmt_dos_clients_io::gmt_m1::{assembly::M1ModeCoefficients, M1ModeShapes};
use interface::{Data, Read, Update, Write};
use serde::{Deserialize, Serialize};

use crate::m1_parameters::M1_N_RAW_MODE;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct SingularModes {
    mode_nodes: Vec<Vec<f64>>,
    actuator_nodes: Vec<Vec<f64>>,
    raw_modes: Vec<f64>,
    modes: Vec<f64>,
    mode_2_force: Vec<f64>,
    shape: (usize, usize),
}

#[derive(Debug, Default, Clone)]
pub struct M1BendingModes {
    modes: Vec<SingularModes>,
    surfaces: Arc<Vec<f64>>,
    coefs: Arc<Vec<f64>>,
}

impl M1BendingModes {
    pub fn new() -> anyhow::Result<Self> {
        let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set!");
        let fem_path = Path::new(&fem_var);
        let modes: Vec<SingularModes> = serde_pickle::from_reader(
            &mut File::open(fem_path.join("m1_sms.pkl"))?,
            Default::default(),
        )?;
        Ok(Self {
            modes,
            ..Default::default()
        })
    }
}
impl Update for M1BendingModes {
    fn update(&mut self) {
        let mut ns_acc = 0;
        self.coefs = Arc::new(
            self.modes
                .iter()
                .flat_map(|mode| {
                    let (ns, na) = mode.shape;
                    let mat = faer::mat::from_column_major_slice::<f64>(&mode.raw_modes, ns, na);
                    let deltas = &self.surfaces[ns_acc..ns_acc + ns];
                    ns_acc += ns;
                    let coefs =
                        mat.transpose() * faer::mat::from_column_major_slice::<f64>(deltas, ns, 1);
                    let mut coefs = coefs.col_as_slice(0).to_vec();
                    coefs.extend(vec![0f64; M1_N_RAW_MODE - na]);
                    coefs
                })
                .collect(),
        );
    }
}
impl Read<M1ModeShapes> for M1BendingModes {
    fn read(&mut self, data: Data<M1ModeShapes>) {
        self.surfaces = data.into_arc();
    }
}
impl Write<M1ModeCoefficients> for M1BendingModes {
    fn write(&mut self) -> Option<Data<M1ModeCoefficients>> {
        Some(self.coefs.clone().into())
    }
}
// impl Write<M1ActuatorCommandForces> for M1BendingModes {
// fn write(&mut self) -> Option<Data<M1ActuatorCommandForces>> {
// Some(forces.into())
// }
// }
// impl Read<M1Segment1AxialD> for M1BendingModes {
//     fn read(&mut self, data: Data<M1Segment1AxialD>) {
//         self.axial_d = data.into_arc();
//     }
// }
// impl<const ID: u8> Write<BendingModes<ID>> for M1BendingModes {
//     fn write(&mut self) -> Option<Data<BendingModes<ID>>> {
//         let (ns, na) = self.modes.shape;
//         let surface = faer::mat::from_column_major_slice::<f64>(&self.axial_d, ns, 1);
//         let u = faer::mat::from_column_major_slice::<f64>(&self.modes.modes, ns, na);
//         let c = u.subcols(0, self.n_mode).transpose() * surface;
//         Some(c.col_as_slice(0).into())
//     }
// }
