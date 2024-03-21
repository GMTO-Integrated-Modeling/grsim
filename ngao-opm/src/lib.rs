use interface::{UniqueIdentifier, UID};

pub const N_MODE: usize = 500;
pub const N_ACTUATOR: usize = 675;
pub const ACTUATOR_RATE: usize = 80;

pub mod hdfs;
pub mod modal_to_zonal;

#[derive(UID)]
#[uid(port = 60000)]
pub enum SegmentAD7Piston {}
pub enum MayBeSegmentAD7Piston {}
impl UniqueIdentifier for MayBeSegmentAD7Piston {
    type DataType = Option<Vec<f64>>;
}

#[derive(UID)]
#[uid(port = 60001)]
pub enum SegmentPistonRecon {}

#[derive(UID)]
#[uid(port = 60002)]
pub enum SegmentPistonInt {}

#[derive(UID)]
#[uid(port = 60003)]
pub enum M2modesRecon {}
