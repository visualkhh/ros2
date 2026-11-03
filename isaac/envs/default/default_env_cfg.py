"""DefaultEnvCfg — BaseEnvCfg 그대로 + 사람이 볼 때 설정 (카메라가 로봇을 따라감 · 명령/실제 막대)."""
from __future__ import annotations

from isaaclab.utils import configclass
from isaaclab_visualizers.kit import KitVisualizerCfg

from envs.base_env_cfg import BaseEnvCfg


@configclass
class DefaultEnvCfg(BaseEnvCfg):
    debug_vis = True   # 초록 = 명령 방향 · 파랑 = 실제 방향

    def __post_init__(self):
        super().__post_init__()
        self.sim.visualizer_cfgs = [KitVisualizerCfg(origin_type="asset", origin_track_path="robot",
                                                     eye=(-1.2, -1.2, 1.0), lookat=(0.0, 0.0, 0.0))]
