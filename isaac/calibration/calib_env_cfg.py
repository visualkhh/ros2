"""calibration env 설정 — 기본 세상(BaseEnvCfg: 로봇 1대 · 평평 바닥 · 480/20 Hz) + 바퀴 모델 고정값 + (선택) 방·장애물.

학습 env와 같은 로봇·물리이고, 다른 점만 여기 있다: 랜덤 없음 · 바퀴 모델은 `--fit` 값 그대로(`WheelModelCfg`, 기본 = raw plant).
"""
from __future__ import annotations

from isaaclab.utils import configclass

from envs.base_env_cfg import OBSTACLE_FRICTION
from envs.base_env_cfg import BaseEnvCfg


@configclass
class WheelModelCfg:
    """sim 입력에 얹는 바퀴 모델: 명령 지연 → 좌/우 gain → 데드밴드 → 1차 지연 τ (정책 주기마다).

    기본값 = raw plant (보정 없음). `calibration.fit.wheel_of`가 fit.json에서 채운다.
    """
    tau_s: float = 0.0
    delay_steps: int = 0          # 정책 주기(20 Hz) 단위
    gain_l: float = 1.0
    gain_r: float = 1.0
    deadband: float = 0.0         # [rad/s]


@configclass
class CalibEnvCfg(BaseEnvCfg):
    wheel: WheelModelCfg = WheelModelCfg()
    # 방 + 장애물 (random-world). 0이면 빈 바닥 — 측정은 0
    rooms: int = 0
    room_size: tuple[float, float] = (3.5, 6.0)   # [m] 한 변 (방마다 랜덤)
    room_seed: int = 0
    room_spacing: float = 8.0                     # [m] 방 사이 — 라이다(≈3.5 m)가 옆방을 못 보게
    obstacle_friction: tuple[float, float] = OBSTACLE_FRICTION


@configclass
class RoomsEnvCfg(CalibEnvCfg):
    """random-world: 랜덤 방 8개 (N 키로 순간이동)."""
    rooms: int = 8
