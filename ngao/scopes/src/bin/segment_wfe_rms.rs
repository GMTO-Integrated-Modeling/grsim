use gmt_dos_clients_io::optics::{SegmentWfeRms, WfeRms};
use gmt_dos_clients_scope_client::Scope;

#[tokio::main]
async fn main() {
    let server_ip = "44.235.124.92";
    let server_port = 5006;
    let client_address = "0.0.0.0:0";

    loop {
        Scope::new(server_ip, client_address)
            .signal::<SegmentWfeRms<-9>>(5006)
            .unwrap()
            .signal::<WfeRms<-9>>(5007)
            .unwrap()
            .show();
    }
}
