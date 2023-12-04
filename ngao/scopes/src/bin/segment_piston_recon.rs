use gmt_dos_actors_clients_interface::{units::NM, UniqueIdentifier};
use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::Scope;

pub enum SegmentPistonRecon {}
impl UniqueIdentifier for SegmentPistonRecon {
    type DataType = Vec<f64>;
}

#[tokio::main]
async fn main() {
    let server_ip = "44.235.124.92";
    let server_port = 5002;
    let client_address = "0.0.0.0:0";

    loop {
        Scope::new(server_ip, client_address)
            .signal::<NM<SegmentPistonRecon>>(server_port)
            .unwrap()
            .show();
    }
}
