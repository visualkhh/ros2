"""calibration env 설정 — 로봇 1대 · 평평한 바닥(선택: 방+장애물) · 바퀴 모델 고정값 (Isaac Lab Direct 관례).

학습 env(rl_correct_env_cfg)와 같은 로봇(`assets.tb3_waffle_pi.TB3_CFG`)을 쓰고, 다른 점만 여기 있다:
랜덤 없음 · 시간 초과 없음 · 바퀴 모델은 `--fit`으로 준 값 그대로(`WheelModelCfg`, 기본 = raw plant).
"""
from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

from assets.tb3_waffle_pi import GROUND_FRICTION, OBSTACLE_FRICTION, TB3_CFG
from urdf import robot_spec as spec


@configclass
class WheelModelCfg:
    """sim 입력에 얹는 바퀴 모델: 명령 지연 → 좌/우 gain → 데드밴드 → 1차 지연 τ (정책 주기마다).

    기본값 = raw plant (보정 없음). `calibration.fit.wheel_from_fit`이 fit.json에서 채운다.
    """
    tau_s: float = 0.0
    delay_steps: int = 0          # 정책 주기(20 Hz) 단위
    gain_l: float = 1.0
    gain_r: float = 1.0
    deadband: float = 0.0         # [rad/s]


@configclass
class CalibEnvCfg(DirectRLEnvCfg):
    decimation = round(spec.PHYSICS_HZ / spec.CONTROL_HZ)   # 480 Hz 물리 → 20 Hz 명령 (= cmd_manager)
    episode_length_s = 1e6                                   # 시간 초과 리셋 없음
    action_space = 2          # [v, w] 명령 그대로 (보정은 스크립트가 --policy로 먼저 더함)
    observation_space = 3     # [v_enc, w_enc, w_gyro]
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / spec.PHYSICS_HZ, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1, env_spacing=2.0, replicate_physics=True)
    robot_cfg: ArticulationCfg = TB3_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    terrain = TerrainImporterCfg(
        prim_path="/World/ground", terrain_type="plane",
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GROUND_FRICTION[0],
                                                        dynamic_friction=GROUND_FRICTION[1],
                                                        restitution=0.0, friction_combine_mode="average"))
    wheel: WheelModelCfg = WheelModelCfg()

    # 방 + 장애물 (random-world). 0이면 빈 바닥 — 측정은 0
    rooms: int = 0
    room_size: tuple[float, float] = (3.5, 6.0)   # [m] 한 변 (방마다 랜덤)
    room_seed: int = 0
    room_spacing: float = 8.0                     # [m] 방 사이 — 라이다(≈3.5 m)가 옆방을 못 보게
    obstacle_friction: tuple[float, float] = OBSTACLE_FRICTION

    debug_vis = False   # 창 볼 때 스크립트가 켬 — 명령/실제 막대 + 목표 경로


@configclass
class RoomsEnvCfg(CalibEnvCfg):
    """random-world: 랜덤 방 8개 (N 키로 순간이동)."""
    rooms: int = 8
