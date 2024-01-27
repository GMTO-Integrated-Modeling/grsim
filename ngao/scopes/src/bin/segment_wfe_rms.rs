use gmt_dos_clients_io::optics::{SegmentWfeRms, WfeRms};
use gmt_dos_clients_scope_client::Scope;

#[tokio::main]
async fn main() {
    loop {
        Scope::new()
            .signal::<SegmentWfeRms<-9>>()
            .unwrap()
            .signal::<WfeRms<-9>>()
            .unwrap()
            .show();
    }
}
