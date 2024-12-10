use std::fmt::Display;

use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::model::{Check, SystemFlowChart, Task},
    system::System,
};

use crate::{Dfs, Ltws, Model, Models, Oiwfs, RxyPiston, Sh48};

mod models;
mod sys_io;
pub use models::Meta;

#[derive(Clone)]
pub struct MetaOpticalModel<
    const DFS_CAM_RATE: usize = 1,
    const DFS_FFT_RATE: usize = 1,
    const SH48_RATE: usize = DFS_CAM_RATE,
    const OIWFS_RATE: usize = 1,
    const LTWS_RATE: usize = 1,
> {
    ltws: Actor<Meta<Ltws>, 1, LTWS_RATE>,
    oiwfs: Actor<Meta<Oiwfs>, 1, OIWFS_RATE>,
    sh48: Actor<Meta<Sh48<SH48_RATE>>, 1, SH48_RATE>,
    dfs: Actor<Meta<Dfs<RxyPiston, DFS_CAM_RATE, DFS_FFT_RATE>>, 1, SH48_RATE>,
}

impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    pub fn new() -> anyhow::Result<Self> {
        let models = Models::new();
        Ok(Self {
            ltws: Meta(models.ltws().builder().build()?).into(),
            oiwfs: Meta(models.oiwfs().builder().build()?).into(),
            sh48: Meta(models.sh48::<SH48_RATE>().builder().build()?).into(),
            dfs: Meta(
                models
                    .dfs::<RxyPiston, DFS_CAM_RATE, DFS_FFT_RATE>()
                    .builder()
                    .build()?,
            )
            .into(),
        })
    }
}

impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > Display for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "M.\nO.\nM.")
    }
}

impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > System for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn build(&mut self) -> anyhow::Result<&mut Self> {
        Ok(self)
    }

    fn plain(&self) -> PlainActor {
        let mut plain = PlainActor::default();
        plain.client = self.name();
        plain.inputs_rate = 1;
        plain.outputs_rate = 1;
        plain.inputs = PlainActor::from(&self.ltws).inputs;
        plain.outputs = PlainActor::from(&self.ltws).outputs;
        plain.graph = self.graph();
        plain
    }
}

impl<
        'a,
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > IntoIterator
    for &'a MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    type Item = Box<&'a dyn Check>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(&self.ltws as &dyn Check),
            Box::new(&self.oiwfs as &dyn Check),
            Box::new(&self.sh48 as &dyn Check),
            Box::new(&self.dfs as &dyn Check),
        ]
        .into_iter()
    }
}

impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > IntoIterator
    for Box<MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>>
{
    type Item = Box<dyn Task>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(self.ltws) as Box<dyn Task>,
            Box::new(self.oiwfs) as Box<dyn Task>,
            Box::new(self.sh48) as Box<dyn Task>,
            Box::new(self.dfs) as Box<dyn Task>,
        ]
        .into_iter()
    }
}
