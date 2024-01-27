use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::GridScope;
use interface::{units::NM, UID};

#[derive(UID)]
pub enum SegmentPistonRecon {}

#[derive(UID)]
pub enum SegmentAD7Piston {}

#[derive(UID)]
pub enum SegmentPistonInt {}

#[allow(unreachable_code)]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    loop {
        GridScope::new((2, 2))
            .pin::<SegmentPiston<-9>>((0, 0))?
            .pin::<NM<SegmentPistonRecon>>((0, 1))?
            .pin::<SegmentAD7Piston>((1, 0))?
            .pin::<NM<SegmentPistonInt>>((1, 1))?
            .show();
    }
    Ok(())
}
