use std::sync::Arc;

use interface::{Data, Read, Update, Write};

use crate::SegmentAD7Piston;

#[derive(Debug, Default)]
pub struct HDFS {
    data: Arc<Vec<f64>>,
    scale: Vec<f64>,
}
impl HDFS {
    pub fn new(scale: Vec<f64>) -> Self {
        Self {
            data: Default::default(),
            scale,
        }
    }
}
impl Update for HDFS {}
impl Read<SegmentAD7Piston> for HDFS {
    fn read(&mut self, data: Data<SegmentAD7Piston>) {
        self.data = data.into_arc();
    }
}
impl Write<SegmentAD7Piston> for HDFS {
    fn write(&mut self) -> Option<Data<SegmentAD7Piston>> {
        let data: Vec<_> = self
            .data
            .iter()
            .zip(&self.scale)
            .map(|(x, s)| if x.abs() > 250e-9 { *x * s } else { 0f64 })
            .collect();
        // let mean = data.iter().sum::<f64>() / 7f64;
        // Some(
        //     data.into_iter()
        //         .map(|x| x - mean)
        //         .collect::<Vec<_>>()
        //         .into(),
        // )
        Some(data.into())
    }
}
