use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::{GridScope, Scope};
use interface::{units::NM, UID};

#[derive(UID)]
pub enum SegmentPistonRecon {}

#[derive(UID)]
pub enum SegmentAD7Piston {}

#[derive(UID)]
pub enum SegmentPistonInt {}

const CLIENT_ADDRESS: &str = "0.0.0.0:0";
const SERVER_IP: &str = "44.235.124.92";

#[allow(unreachable_code)]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    loop {
        GridScope::new((2, 2))
            .pin(
                (0, 0),
                Scope::new(SERVER_IP, CLIENT_ADDRESS).signal::<SegmentPiston<-9>>(5001)?,
            )
            .pin(
                (0, 1),
                Scope::new(SERVER_IP, CLIENT_ADDRESS).signal::<NM<SegmentPistonRecon>>(5002)?,
            )
            .pin(
                (1, 0),
                Scope::new(SERVER_IP, CLIENT_ADDRESS).signal::<SegmentAD7Piston>(5003)?,
            )
            .pin(
                (1, 1),
                Scope::new(SERVER_IP, CLIENT_ADDRESS).signal::<NM<SegmentPistonInt>>(5004)?,
            )
            .show();
    }
    Ok(())
}
