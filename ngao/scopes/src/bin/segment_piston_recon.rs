use interface::{units::NM, UniqueIdentifier};
use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::Scope;

pub enum SegmentPistonRecon {}
impl UniqueIdentifier for SegmentPistonRecon {
    type DataType = Vec<f64>;
}

#[tokio::main]
async fn main() {
    loop {
        Scope::new()
            .signal::<NM<SegmentPistonRecon>>()
            .unwrap()
            .show();
    }
}
