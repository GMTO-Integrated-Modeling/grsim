use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::Scope;
use interface::UID;

#[derive(UID)]
pub enum SegmentAD7Piston {}

#[tokio::main]
async fn main() {
    let server_ip = "44.235.124.92";
    let server_port = 5003;
    let client_address = "0.0.0.0:5003";

    loop {
        Scope::new(server_ip, client_address)
            .signal::<SegmentAD7Piston>(server_port)
            .unwrap()
            .show();
    }
}
