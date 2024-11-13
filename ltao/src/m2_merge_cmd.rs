use std::{fs::File, path::Path, sync::Arc};

use gmt_dos_clients_io::{
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{M2GlobalTipTilt, SegmentPiston},
};
use interface::{Data, Read, Update, Write};

use crate::{M2GttToPtt, M2_N_MODE};

#[derive(Debug, Default, Clone)]
pub struct MergeAsmCommand {
    gtt_to_ptt: M2GttToPtt,
    global_tiptilt: Option<Arc<Vec<f64>>>,
    piston: Option<Arc<Vec<f64>>>,
    modes: Vec<f64>,
}
impl MergeAsmCommand {
    pub fn new() -> anyhow::Result<Self> {
        let data_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples")
            .join("m2_gtt-to-modes");

        let gtt_to_ptt = serde_pickle::from_reader(
            &mut File::open(data_path.join("m2_gtt_to_ptt.pkl"))?,
            Default::default(),
        )?;
        Ok(Self {
            gtt_to_ptt,
            modes: vec![0f64; 7 * M2_N_MODE],
            ..Default::default()
        })
    }
}
impl Update for MergeAsmCommand {
    fn update(&mut self) {
        if let Some(gtt) = self.global_tiptilt.as_ref() {
            let ptt = self.gtt_to_ptt.transfrom(gtt);
            self.modes
                .chunks_mut(M2_N_MODE)
                .zip(ptt.chunks(ptt.len() / 7))
                .for_each(|(m, ptt)| {
                    m.iter_mut().zip(ptt).for_each(|(m, ptt)| *m += *ptt);
                });
        }
        if let Some(piston) = self.piston.as_ref() {
            // self.modes
            //     .chunks_mut(M2_N_MODE)
            //     .zip(piston.chunks(M2_N_MODE))
            //     .for_each(|(m, p)| m.iter_mut().zip(p).for_each(|(m, p)| *m += *p));
            self.modes
                .chunks_mut(M2_N_MODE)
                .zip(piston.iter())
                .for_each(|(m, p)| m[0] += *p);
        }
    }
}
impl Read<SegmentPiston> for MergeAsmCommand {
    fn read(&mut self, data: Data<SegmentPiston>) {
        self.piston = Some(data.into_arc());
    }
}
impl Read<M2ASMAsmCommand> for MergeAsmCommand {
    fn read(&mut self, data: Data<M2ASMAsmCommand>) {
        self.modes = data.into_arc().as_slice().to_vec();
    }
}
impl Read<M2GlobalTipTilt> for MergeAsmCommand {
    fn read(&mut self, data: Data<M2GlobalTipTilt>) {
        self.global_tiptilt = Some(data.into_arc());
    }
}
impl Write<M2ASMAsmCommand> for MergeAsmCommand {
    fn write(&mut self) -> Option<Data<M2ASMAsmCommand>> {
        Some(self.modes.clone().into())
    }
}
