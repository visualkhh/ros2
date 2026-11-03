"""rl_correct v2 config — 우리 Waffle Pi(URDF)로 '실제로 관측되는 것만' 보정 (documents/isaac/task-spec.md §3)."""
from __future__ import annotations

import math

import isaaclab.terrains as terrain_gen
from isaaclab.envs import mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.utils import configclass

from envs.base_env_cfg import BaseEnvCfg

from envs.base_env_cfg import GROUND as _GROUND  # noqa: E402


def make_terrain(slope_deg: float, uneven: bool) -> TerrainImporterCfg:
    """바닥 — Isaac Lab 지형 생성기 (anymal_c_direct Rough와 같은 방식). 둘 다 끄면 평면.

    8 m 칸을 격자로 깔고 칸마다 종류·난이도 랜덤 (curriculum 없음): 평평 · 경사(피라미드 오르막/내리막, 0~slope_deg)
    · 요철(낮은 턱 1~3 cm). env는 칸 가운데(1 m 평평한 판)에서 출발 — 한 칸에 로봇 여럿이 겹쳐도 서로 충돌 안 함.
    """
    sub = {"flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.2)}
    if slope_deg > 0:
        rng = (0.0, math.radians(slope_deg))
        sub["slope_up"] = terrain_gen.HfPyramidSlopedTerrainCfg(proportion=0.2, slope_range=rng, platform_width=1.0)
        sub["slope_down"] = terrain_gen.HfInvertedPyramidSlopedTerrainCfg(proportion=0.2, slope_range=rng, platform_width=1.0)
    if uneven:
        sub["bumps"] = terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.4, obstacle_height_mode="fixed", obstacle_width_range=(0.30, 0.55),
            obstacle_height_range=(0.01, 0.03), num_obstacles=30, platform_width=1.0)
    if len(sub) == 1:
        return TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane", physics_material=_GROUND)
    gen = TerrainGeneratorCfg(size=(8.0, 8.0), border_width=4.0, num_rows=8, num_cols=8,
                              horizontal_scale=0.05, vertical_scale=0.002, slope_threshold=0.75,
                              color_scheme="height", sub_terrains=sub)
    return TerrainImporterCfg(prim_path="/World/ground", terrain_type="generator", terrain_generator=gen,
                              physics_material=_GROUND, visual_material=None)


@configclass
class EventCfg:
    """env마다 바퀴 마찰 랜덤 (anymal_c_direct EventCfg와 같은 방식). 바닥은 고정 · 접촉 마찰 = 바퀴와 바닥 평균.

    바퀴 0.7~1.1 ↔ 바닥 (0.8, 0.7) 평균 = 0.75~0.95 — 예전 'env별 바닥 0.5~0.9 × 바퀴 1.0'과 같은 접촉 범위.
    """
    wheel_friction = EventTerm(
        func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="wheel_.*_link"),
                "static_friction_range": (0.7, 1.1), "dynamic_friction_range": (0.7, 1.1),
                "restitution_range": (0.0, 0.0), "num_buckets": 64, "make_consistent": True})


@configclass
class RlCorrectEnvCfg(BaseEnvCfg):
    """기본 세상(BaseEnvCfg: 로봇 · 480/20 Hz) + 로봇 수천 대 · 지형 · 바퀴 랜덤 · 관측 6 / 행동 2."""
    episode_length_s = 10.0
    random_yaw = True
    action_space = 2       # [dV, dW]  (ONNX 출력은 action_scale 곱한 물리값)
    observation_space = 6  # [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]
    state_space = 0


    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=2048, env_spacing=3.0, replicate_physics=True)
    terrain: TerrainImporterCfg = make_terrain(0.0, False)   # train.py가 --slope/--uneven으로 다시 만듦
    events: EventCfg | None = None                          # train.py가 --friction(기본)이면 EventCfg()

    # 정책 출력 [-1, 1] → 보정량 (ONNX export도 같은 값으로 곱해 물리값을 낸다)
    action_scale = (0.10, 0.60)  # [m/s, rad/s]

    # 에피소드마다 무작위 (실제에서 엔코더·자이로로 보이는 오차만)
    # 실기기 맞춤 (2026-11-02 측정: artifacts/calibration/20261102_140931-02-real − …144928-sim_480, sim raw 대비 차이)
    #   gain L/R 0.9927/0.9938 (real/sim 비율) · 명령 지연 +2 스텝(0.1 s) · τ 차이 0 · 데드밴드 0 → 아래 범위의 중심
    #   하드웨어가 바뀌면 다시 측정(sensor-sync-real + report)해서 고치고 sensor-sync-sim으로 맞는지 확인
    tau_range = (0.01, 0.10)       # [s] 바퀴별 1차 지연 (τ 차이 0 ± 0.1, 최소 0.01)
    gain_range = (0.90, 1.10)      # 바퀴별 이득 (좌우 비대칭)
    gain_center = (0.9927, 0.9938) # 좌/우 gain 중심 (실측) — gain_range 샘플에 곱함
    deadband_range = (0.0, 0.3)    # [rad/s] 이보다 작은 바퀴 명령은 안 돎
    # documents/isaac/task-spec.md §3 기준: 명령·관측 각각 0~4 스텝 (0~200 ms)
    cmd_delay_steps = (0, 4)       # 명령 지연 [×50 ms] (실측 2 ± 2)
    obs_delay_steps = (0, 4)       # Pi → PC (odom·imu)
    sep_nominal_err = 0.08         # 드라이버 기준 간격 vs 실제 유효 간격 (회전 오차 → 자이로로만 보임)
    radius_nominal_err = 0.03      # 기준 반지름 오차 (직진 거리 — 관측 안 됨, 강건성용)
    enc_noise = 0.005
    gyro_noise = 0.01
    # 자이로 영점 오차 [rad/s] — 에피소드마다 ±gyro_bias 고정값을 w_gyro에 더함 (OpenCR 부팅 보정이 덜 된 경우).
    # 정책이 w_enc와 비교해 영점을 무시하도록. 11-02 실기기 −0.257은 부팅 중 움직인 고장 — OpenCR 재부팅으로 고친다.
    gyro_bias = 0.05

    # 바닥은 make_terrain · 바퀴 마찰 랜덤은 EventCfg · 바퀴/캐스터 기본 마찰은 BaseEnvCfg
    slope_deg = 0.0    # 기록용 (ranges.json) — 실제 바닥은 terrain
    uneven = False

    # 명령: hold_range 초마다 바뀜 (텔레오퍼 키 · 부드러운 곡선 · 정지)
    hold_range = (0.5, 2.0)
    p_teleop, p_smooth = 0.6, 0.3   # 나머지 = 정지

    # 보상
    ang_weight = 0.5
    action_penalty = 0.01
    smooth_penalty = 0.05
