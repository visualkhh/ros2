"""기본 세상 설정 — 로봇 1대 · 평평한 바닥 · 조명 · 480 Hz 물리 / 20 Hz 명령. 아무 규칙 없음 (보상 0 · 종료 없음).

task cfg가 상속해서 바꾸는 것: scene.num_envs · terrain(지형) · 관측/행동 크기 · 자기 랜덤 범위.
로봇은 assets 한 곳(`TB3_CFG`) — 다른 로봇이면 robot_cfg만 바꾼다.
"""
from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

from assets.tb3_waffle_pi import CASTER_FRICTION, GROUND_FRICTION, TB3_CFG, WHEEL_FRICTION
from urdf import robot_spec as spec

GROUND = sim_utils.RigidBodyMaterialCfg(static_friction=GROUND_FRICTION[0], dynamic_friction=GROUND_FRICTION[1],
                                        restitution=0.0, friction_combine_mode="average")


@configclass
class BaseEnvCfg(DirectRLEnvCfg):
    decimation = round(spec.PHYSICS_HZ / spec.CONTROL_HZ)   # 480 Hz 물리 → 20 Hz 명령 (= cmd_manager)
    episode_length_s = 1e6       # 시간 초과 없음 (학습 task는 자기 에피소드 길이로)
    action_space = 2             # [v, w] 명령 그대로
    observation_space = 3        # [v_enc, w_enc, w_gyro]
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / spec.PHYSICS_HZ, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1, env_spacing=2.0, replicate_physics=True)
    robot_cfg: ArticulationCfg = TB3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    terrain: TerrainImporterCfg = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane", physics_material=GROUND)
    wheel_friction = WHEEL_FRICTION
    caster_friction = CASTER_FRICTION
    random_yaw = False          # 리셋 때 방향 랜덤 (학습) · False면 0
    debug_vis = False           # 창 볼 때 스크립트가 켬 — 명령(초록)/실제(파랑) 막대 + 목표 경로(노란 점, path_w)
