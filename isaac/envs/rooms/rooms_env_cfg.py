"""RoomsEnvCfg — 기본 환경 + 랜덤 방(벽 4 + 장애물 3~6개, 방마다 크기 랜덤)."""
from __future__ import annotations

from isaaclab.utils import configclass

from envs.base_env_cfg import OBSTACLE_FRICTION
from envs.default.default_env_cfg import DefaultEnvCfg


@configclass
class RoomsEnvCfg(DefaultEnvCfg):
    rooms: int = 8
    room_size: tuple[float, float] = (3.5, 6.0)   # [m] 한 변 (방마다 랜덤)
    room_seed: int = 0
    room_spacing: float = 8.0                     # [m] 방 사이 — 라이다(≈3.5 m)가 옆방을 못 보게
    obstacle_friction: tuple[float, float] = OBSTACLE_FRICTION
