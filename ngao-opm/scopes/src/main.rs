use gmt_dos_clients_io::optics::{SegmentWfeRms, WfeRms};
use gmt_dos_clients_scope_client::Scope;

#[allow(unreachable_code)]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
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
