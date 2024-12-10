use gmt_dos_actors::{
    actor::Actor,
    system::{SystemInput, SystemOutput},
};

use crate::{
    agws_parameters::SH48_INT, ltws_parameter::LTWS_INT, Dfs, Ltws, Oiwfs, RxyPiston, Sh48,
};

use super::{Meta, MetaOpticalModel};

impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > SystemInput<Meta<Ltws>, 1, LTWS_RATE>
    for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn input(&mut self) -> &mut Actor<Meta<Ltws>, 1, LTWS_RATE> {
        &mut self.ltws
    }
}
impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > SystemInput<Meta<Oiwfs>, 1, OIWFS_RATE>
    for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn input(&mut self) -> &mut Actor<Meta<Oiwfs>, 1, OIWFS_RATE> {
        &mut self.oiwfs
    }
}
impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > SystemInput<Meta<Sh48<SH48_RATE>>, 1, SH48_RATE>
    for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn input(&mut self) -> &mut Actor<Meta<Sh48<SH48_RATE>>, 1, SH48_RATE> {
        &mut self.sh48
    }
}

impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > SystemInput<Meta<Dfs<RxyPiston, DFS_CAM_RATE, DFS_FFT_RATE>>, 1, SH48_RATE>
    for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn input(
        &mut self,
    ) -> &mut Actor<Meta<Dfs<RxyPiston, DFS_CAM_RATE, DFS_FFT_RATE>>, 1, SH48_RATE> {
        &mut self.dfs
    }
}

impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > SystemOutput<Meta<Ltws>, 1, LTWS_RATE>
    for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn output(&mut self) -> &mut Actor<Meta<Ltws>, 1, LTWS_RATE> {
        &mut self.ltws
    }
}
impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > SystemOutput<Meta<Oiwfs>, 1, OIWFS_RATE>
    for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn output(&mut self) -> &mut Actor<Meta<Oiwfs>, 1, OIWFS_RATE> {
        &mut self.oiwfs
    }
}
impl<const DFS_CAM_RATE: usize, const DFS_FFT_RATE: usize>
    SystemOutput<Meta<Sh48<SH48_INT>>, 1, SH48_INT>
    for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_INT>
{
    fn output(&mut self) -> &mut Actor<Meta<Sh48<SH48_INT>>, 1, SH48_INT> {
        &mut self.sh48
    }
}

impl<
        const DFS_CAM_RATE: usize,
        const DFS_FFT_RATE: usize,
        const SH48_RATE: usize,
        const OIWFS_RATE: usize,
        const LTWS_RATE: usize,
    > SystemOutput<Meta<Dfs<RxyPiston, DFS_CAM_RATE, DFS_FFT_RATE>>, 1, SH48_RATE>
    for MetaOpticalModel<DFS_CAM_RATE, DFS_FFT_RATE, SH48_RATE, OIWFS_RATE, LTWS_RATE>
{
    fn output(
        &mut self,
    ) -> &mut Actor<Meta<Dfs<RxyPiston, DFS_CAM_RATE, DFS_FFT_RATE>>, 1, SH48_RATE> {
        &mut self.dfs
    }
}
