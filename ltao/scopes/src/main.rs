use gmt_dos_clients_io::optics::{SegmentWfeRms, WfeRms};
use gmt_dos_clients_scope_client::Scope;
use std::env;

#[allow(unreachable_code)]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env::set_var("SCOPE_SERVER_IP", "44.235.124.92");
    loop {
        Scope::new()
            .signal::<SegmentWfeRms<-9>>()
            .unwrap()
            .signal::<WfeRms<-9>>()
            .unwrap()
            .show();
    }
    Ok(())
}
