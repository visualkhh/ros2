"""rl_correction_waypoint 설정 — 랜덤 웨이포인트를 이탈 없이 따라가기 (기본 제어기 + 정책 보정). 로봇과 상관없음."""
from __future__ import annotations

import isaaclab.terrains as terrain_gen
from isaaclab.envs import ViewerCfg, mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.utils import configclass

from envs.base_env_cfg import BaseEnvCfg

from envs.base_env_cfg import GROUND as _GROUND  # noqa: E402


def make_terrain(uneven: bool) -> TerrainImporterCfg:
    """바닥 — Isaac Lab 지형 생성기 (anymal_c_direct Rough와 같은 방식). uneven이 꺼지면 평면.

    8 m 칸을 격자로 깔고 칸마다 종류 랜덤 (curriculum 없음): 평평 · 요철(낮은 턱 1~3 cm). 언덕(경사)은 없다.
    env는 칸 가운데(1 m 평평한 판)에서 출발 — 한 칸에 로봇 여럿이 겹쳐도 서로 충돌 안 함.
    """
    sub = {"flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.2)}
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
class RlCorrectionWaypointEnvCfg(BaseEnvCfg):
    """기본 세상(BaseEnvCfg: 480/20 Hz) + 로봇 수천 대 · 지형 · 바퀴 랜덤. 관측 10 / 행동 2.

    로봇은 BaseEnvCfg.robot (기본 robots.DEFAULT = 이 폴더 · 바꾸기: env.robot=<이름|폴더>) — 그 로봇의 robot_cfg.py가
    바퀴 숫자 · 실측 응답(RESPONSE_*)을 채운다. 실측 응답이 랜덤 범위의 중심, 여기는 폭만.
    로봇 corrector goal 모드와 같은 약속: 관측 [gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_a0, prev_a1]
    (시작 자세 기준, 위치·방향 = 엔코더 거리 + 자이로 방향 적분) · 기본 제어기 식 · 행동 [-1, 1] × action_scale.
    """
    episode_length_s = 40.0        # 웨이포인트 num_waypoints개 (각 ≤ 1.5 m)를 다 돌 시간
    random_yaw = True
    action_space = 2               # [Δv, Δw] 정책 출력 [-1, 1] (corrector가 × action_scale)
    observation_space = 10
    state_space = 0

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=512, env_spacing=3.0, replicate_physics=True)   # --num_envs
    viewer: ViewerCfg = ViewerCfg(eye=(2.0, -2.0, 1.5), lookat=(0.0, 0.0, 0.0), origin_type="env", env_index=0)   # 창: env 0 로봇 근처 (env마다 8 m 칸이라 멀리 흩어져 있다)
    terrain: TerrainImporterCfg = make_terrain(True)   # env가 장면 만들 때 uneven으로 다시 만든다
    events: EventCfg | None = EventCfg()                    # env마다 바퀴 마찰 랜덤 — 끄려면 env.events=null
    uneven = True                  # 평평 + 요철 칸 (턱 1~3 cm) · env.uneven=false = 평면만
    # 마찰 패치 — env마다 출발점 둘레에 얇은 판 (노랑 = 끈적, 파랑 = 미끄럼). 바퀴와 평균 결합. env.friction_patches=0 = 끔
    friction_patches = 8              # env마다 개수 (반은 끈적 · 반은 미끄럼, 랜덤)
    patch_radius = (0.3, 3.0)         # [m] 출발점에서 거리 (웨이포인트 경로가 지나가는 범위)
    patch_size = (0.4, 0.9)           # [m] 한 변
    sticky_friction = (1.10, 0.90)    # (static, dynamic)
    slippery_friction = (0.20, 0.15)

    # 스크린샷 — snap_s초마다 snap_dir/<시각>.png + snap_dir/latest.png(덮어씀). 0 = 끔. run-training.sh --snap SEC가 채운다
    snap_s = 0.0
    snap_dir = ""

    # 웨이포인트 (command.py) — 시작 자세 기준, 앞 점에서 방향 turn_range만큼 꺾고 dist_range만큼
    num_waypoints = 4
    waypoint_dist = (0.5, 1.5)     # [m]
    waypoint_turn = (-3.14159, 3.14159)   # [rad]

    # 기본 제어기 (= rl_correction corrector goal 모드 파라미터, 바꾸면 같이)
    k_dist = 1.0
    k_heading = 2.0
    turn_thresh = 0.5              # [rad] 이보다 방향이 틀어지면 제자리 회전 먼저
    goal_tol = 0.05                # [m] 도착 판정 (추정 위치 기준)

    # 정책 출력 [-1, 1] → 보정량 (로봇 corrector의 action_scale_lin/ang과 같아야 한다)
    action_scale = (0.10, 0.60)    # [m/s, rad/s]

    # 에피소드마다 무작위 — 중심 = 로봇 실측 응답 (robot_cfg.py RESPONSE_*), 여기는 폭만
    tau_width = 0.10               # [s] τ = 실측 ± 이만큼 (최소 tau_min)
    tau_min = 0.01
    gain_range = (0.90, 1.10)      # 바퀴별 이득 — 실측 좌/우 gain에 곱함
    delay_width = 2                # 명령 지연 = 실측 ± 이만큼 [20 Hz 스텝] (최소 0)
    deadband_range = (0.0, 0.3)    # [rad/s] (실측 데드밴드 + 이 범위)
    obs_delay_steps = (0, 4)       # 센서 지연 (odom · imu)
    sep_nominal_err = 0.08         # 드라이버 기준 간격 vs 실제 유효 간격 (회전 오차 → 자이로로만 보임)
    radius_nominal_err = 0.03      # 기준 반지름 오차 (직진 거리 — 관측 안 됨)
    enc_noise = 0.005
    gyro_noise = 0.01
    gyro_bias = 0.005              # [rad/s] ± — 방향을 자이로 적분으로 보므로 작게 (OpenCR 부팅 보정 후 실측 ≈ 0.0005)

    # 보상 (채점은 sim의 진짜 위치로)
    w_cross = 2.0                  # 경로(앞 점 → 다음 점 직선)에서 벗어난 거리 [m]
    w_heading = 0.5                # 직선 방향과 몸체 방향 차이 [rad] (전진 중일 때만)
    w_progress = 2.0               # 목표에 가까워진 거리 [m]
    reach_bonus = 1.0              # 웨이포인트 도착
    w_reach_err = 5.0              # 도착 판정 순간 진짜 위치와 목표의 거리 [m]
    fail_cross = 1.0               # [m] 경로에서 이만큼 벗어나면 실패로 끝냄
    fail_penalty = 5.0
    action_penalty = 0.01
    smooth_penalty = 0.05
