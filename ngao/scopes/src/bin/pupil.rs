use gmt_dos_clients_io::optics::Wavefront;
use gmt_dos_clients_scope_client::Shot;
use interface::UID;

#[tokio::main]
async fn main() {
    let server_ip = "44.235.124.92";
    let server_port = 5005;
    let client_address = "0.0.0.0:0";

    loop {
        Shot::new(server_ip, client_address)
            .signal::<Wavefront>(server_port)
            .unwrap()
            .show();
    }
}
