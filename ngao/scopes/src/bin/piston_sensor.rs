use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::Scope;
use interface::UID;

#[derive(UID)]
pub enum SegmentAD7Piston {}

#[tokio::main]
async fn main() {
    loop {
        Scope::new()
            .signal::<SegmentAD7Piston>()
            .unwrap()
            .show();
    }
}
