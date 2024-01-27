use gmt_dos_clients_io::optics::Wavefront;
use gmt_dos_clients_scope_client::Shot;
use interface::UID;

#[tokio::main]
async fn main() {
    loop {
        Shot::new()
            .signal::<Wavefront>()
            .unwrap()
            .show();
    }
}
