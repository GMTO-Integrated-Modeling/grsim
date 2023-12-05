use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::GridScope;
use interface::{units::NM, UID};

#[derive(UID)]
pub enum SegmentPistonRecon {}

#[derive(UID)]
pub enum SegmentAD7Piston {}

#[derive(UID)]
pub enum SegmentPistonInt {}

const SERVER_IP: &str = "44.235.124.92";

#[allow(unreachable_code)]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    loop {
        GridScope::new((2, 2), SERVER_IP)
            .pin::<SegmentPiston<-9>>((0, 0), 5001)?
            .pin::<NM<SegmentPistonRecon>>((0, 1), 5002)?
            .pin::<SegmentAD7Piston>((1, 0), 5003)?
            .pin::<NM<SegmentPistonInt>>((1, 1), 5004)?
            .show();
    }
    Ok(())
}
