use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::Scope;

#[tokio::main]
async fn main() {
    let server_ip = "44.235.124.92";
    let server_port = 5003;
    let client_address = "0.0.0.0:0";

    loop {
        Scope::new(server_ip, client_address)
            .signal::<SegmentPiston<-9>>(server_port)
            .unwrap()
            .show();
    }
}
