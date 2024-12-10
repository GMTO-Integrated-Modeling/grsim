use std::sync::Arc;

use faer::Mat;
use gmt_dos_clients_io::optics::M2GlobalTipTilt;
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use serde::{Deserialize, Serialize};

use crate::m2_parameters::M2_N_MODE;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct M2GttToPtt {
    mat: Mat<f64>,
    data: Arc<Vec<f64>>,
}
impl M2GttToPtt {
    pub fn new(mat: Mat<f64>) -> Self {
        Self {
            mat,
            data: Arc::new(vec![0f64; 2]),
        }
    }
    pub fn transform(&self, gtt: &[f64]) -> Vec<f64> {
        let ptt = self.mat.as_ref() * faer::mat::from_column_major_slice::<f64>(gtt, gtt.len(), 1);
        ptt.col_as_slice(0).to_vec()
    }
}
impl Update for M2GttToPtt {}
impl Read<M2GlobalTipTilt> for M2GttToPtt {
    fn read(&mut self, data: Data<M2GlobalTipTilt>) {
        self.data = data.into_arc()
    }
}
impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for M2GttToPtt {
    fn write(&mut self) -> Option<Data<U>> {
        // dbg!(ptt.shape());
        Some(
            self.transform(&self.data)
                .chunks(3)
                .flat_map(|ptt| {
                    let mut modes = ptt.to_vec();
                    modes.extend(vec![0f64; M2_N_MODE - 3]);
                    modes
                })
                .collect::<Vec<f64>>()
                .into(),
        )
    }
}
