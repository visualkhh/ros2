"""DefaultEnvCfg — BaseEnvCfg 그대로 + 명령/실제 막대 켬. 카메라는 고정하지 않는다 (마우스 휠·드래그로 자유, drive의 F = 로봇 쪽으로)."""
from __future__ import annotations

from isaaclab.utils import configclass

from envs.base_env_cfg import BaseEnvCfg


@configclass
class DefaultEnvCfg(BaseEnvCfg):
    debug_vis = True   # 초록 = 명령 방향 · 파랑 = 실제 방향
