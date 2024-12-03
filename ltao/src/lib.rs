pub const N_ACTUATOR: usize = 675;
pub const M1_N_MODE: usize = 27;
pub const M2_N_MODE: usize = 224;
const AGWS_N_GS: usize = 3;

#[derive(UID)]
pub enum M1RbmM2modes {}

pub mod kernels;
mod m1_bending_modes;
mod m2_gtt_to_ptt;
mod m2_merge_cmd;
mod modal_to_zonal;
mod model;

use interface::UID;
pub use m1_bending_modes::M1BendingModes;
pub use m2_gtt_to_ptt::M2GttToPtt;
pub use m2_merge_cmd::MergeAsmCommand;
pub use modal_to_zonal::ModalToZonal;
pub use model::*;
