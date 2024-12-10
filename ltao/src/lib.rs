use interface::UID;

/// M1 parameters
pub mod m1_parameters {
    pub const M1_N_MODE: usize = 27;
    pub static BENDING_MODES: &str = "20240401_1605_m1_bending_modes";
    pub const M1_N_RAW_MODE: usize = 335;
    pub static RAW_BENDING_MODES: &str = "20240401_1605_m1_raw_bending_modes";
    pub static M1_MODE_TO_FORCE: &str = "20240401_1605_m1_mode_to_force.mat";
    pub const M1_TRUSS_PROJECTION: bool = true;
    pub const M1_ACTUATOR_RATE: usize = 80;
}
/// M2 parameters
pub mod m2_parameters {
    pub const M2_N_MODE: usize = 224;
    pub static ASMS_MODES: &str = "asms_eigenmodes_wz123_80pc6";
    pub const ASM_N_ACTUATOR: usize = 675;
    pub static ASMS_INFLUENCE_FUNCTIONS: &str = "asms_ifs_gmt-fem";
    pub static M2_MODE_TO_FORCE: &str = "asms_eigenmodes_wz123_80pc6.mat";
}
/// AGWS paramters
pub mod agws_parameters {
    pub const AGWS_N_GS: usize = 3;
    pub const DFS_CAM_INT: usize = 80;
    pub const DFS_FFT_INT: usize = 500;
    pub const SH48_INT: usize = 40_000;
}
/// LTWS parameters bsys
pub mod ltws_parameter {
    pub const LTWS_INT: usize = 16;
}
/// OIWFS parameters
pub mod oiwfs_parameter {
    pub const OIWFS_INT: usize = 16;
}

#[derive(UID)]
pub enum M1RbmM2modes {}

pub mod kernels;
mod m1_bending_modes;
mod m2_gtt_to_ptt;
mod m2_merge_cmd;
mod meta_optical_model;
mod modal_to_zonal;
mod model;

pub use m1_bending_modes::M1BendingModes;
pub use m2_gtt_to_ptt::M2GttToPtt;
pub use m2_merge_cmd::MergeAsmCommand;
pub use meta_optical_model::{Meta, MetaOpticalModel};
pub use modal_to_zonal::ModalToZonal;
pub use model::*;

pub mod meta {
    use super::agws_parameters::{DFS_CAM_INT, DFS_FFT_INT, SH48_INT};
    pub type MLtws = crate::Meta<crate::Ltws>;
    pub type MOiwfs = crate::Meta<crate::Oiwfs>;
    pub type MSh48 = crate::Meta<crate::Sh48<SH48_INT>>;
    pub type MDfs = crate::Meta<crate::Dfs<crate::RxyPiston, DFS_CAM_INT, DFS_FFT_INT>>;
}
