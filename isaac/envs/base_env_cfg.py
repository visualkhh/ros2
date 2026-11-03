"""기본 세상 설정 — 로봇 1대 · 평평한 바닥 · 조명 · 480 Hz 물리 / 20 Hz 명령. 아무 규칙 없음 (보상 0 · 종료 없음).

task cfg가 상속해서 바꾸는 것: scene.num_envs · terrain(지형) · 관측/행동 크기 · 자기 랜덤 범위.
로봇은 이름(`robot`)만 — cfg가 만들어질 때 그 로봇 폴더의 robot_cfg.py로 채운다 (robot_cfg · 바퀴 숫자 · 마찰).
다른 로봇: `XxxEnvCfg(robot="<이름 또는 폴더>")` · ./run-world.sh --robot. 기본 로봇 이름은 robots.DEFAULT 한 곳.
"""
from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from dataclasses import MISSING

from isaaclab.utils import configclass

import robots

CONTROL_HZ = 20.0      # 명령 주기 = cmd_manager mux 20 Hz = 정책 주기 (calibration report · dummy도 20 — 바꾸면 같이)
# 물리 계산 주기. 120/240 Hz는 제자리 회전 때 바퀴가 바닥에 붙었다 미끄러졌다 해서 바퀴 속도가 떨림
# (정상 회전 구간 표준편차 120 Hz 0.62 · 240 Hz 0.15 · 360 Hz 0.11 · 480 Hz 0.027 · real 0.018 rad/s, 2026-11-02 실측 비교)
PHYSICS_HZ = 480.0

GROUND_FRICTION = (0.8, 0.7)     # 바닥 (static, dynamic) — 로봇이 아니라 세상 쪽
OBSTACLE_FRICTION = (0.7, 0.5)   # 장애물
GROUND = sim_utils.RigidBodyMaterialCfg(static_friction=GROUND_FRICTION[0], dynamic_friction=GROUND_FRICTION[1],
                                        restitution=0.0, friction_combine_mode="average")


@configclass
class BaseEnvCfg(DirectRLEnvCfg):
    decimation = round(PHYSICS_HZ / CONTROL_HZ)   # 480 Hz 물리 → 20 Hz 명령 (= cmd_manager)
    episode_length_s = 1e6       # 시간 초과 없음 (학습 task는 자기 에피소드 길이로)
    action_space = 2             # [v, w] 명령 그대로
    observation_space = 3        # [v_enc, w_enc, w_gyro]
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / PHYSICS_HZ, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1, env_spacing=2.0, replicate_physics=True)
    terrain: TerrainImporterCfg = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane", physics_material=GROUND)
    # 로봇 — 이름(또는 폴더 경로)만 정하면 __post_init__이 그 폴더의 robot_cfg.py에서 아래를 채운다
    robot: str = robots.DEFAULT
    robot_prim_path: str = "{ENV_REGEX_NS}/Robot"
    robot_cfg: ArticulationCfg = MISSING         # USD · 바퀴 모터
    wheel_radius: float = MISSING                # 드라이버 식 숫자
    wheel_sep: float = MISSING
    max_wheel: float = MISSING
    max_lin: float = MISSING                     # [m/s] 명령 한계 (드라이버 · corrector)
    max_ang: float = MISSING                     # [rad/s]
    diag_radius: float = MISSING                 # [m] 텔레오퍼 사선 원호
    # 실측 바퀴 응답 (calibration: sim raw 대비 차이) — 학습 task가 랜덤 범위의 중심으로
    response_gain_lr: tuple[float, float] = MISSING
    response_delay_steps: int = MISSING
    response_tau_s: float = MISSING
    response_deadband: float = MISSING
    wheel_friction: tuple[float, float] = MISSING
    caster_friction: tuple[float, float] = MISSING
    random_yaw = False          # 리셋 때 방향 랜덤 (학습) · False면 0
    debug_vis = False           # 창 볼 때 스크립트가 켬 — 명령(초록)/실제(파랑) 막대 + 목표 경로(노란 점, path_w)

    def __post_init__(self):
        robots.use(self, self.robot)
