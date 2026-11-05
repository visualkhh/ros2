"""rl_nav_correction 설정 — 명령 (v, w)를 몸체가 그대로 따라가게 하는 보정 [Δv, Δw] (nav2 컨트롤러 뒤 노드용). 로봇과 상관없음."""
from __future__ import annotations

from isaaclab.envs import ViewerCfg, mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from envs.base_env_cfg import BaseEnvCfg
from training.rl_nav_correction.ground import GroundCfg


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
class RlNavCorrectionEnvCfg(BaseEnvCfg):
    """기본 세상(BaseEnvCfg: 480/20 Hz) + 로봇 수천 대 · 지형 · 바퀴 랜덤. 관측 6 / 행동 2.

    로봇은 BaseEnvCfg.robot (기본 robots.DEFAULT = 이 폴더 · 바꾸기: env.robot=<이름|폴더>) — 그 로봇의 robot_cfg.py가
    바퀴 숫자 · 실측 응답(RESPONSE_*)을 채운다. 실측 응답이 랜덤 범위의 중심, 여기는 폭만.
    ml rl_nav_correction 노드와 같은 약속: 관측 [v_cmd, w_cmd, v_odom, w_odom, w_gyro, t_change] (원래 단위 — 정규화 없음) ·
    행동 [-1, 1] × action_scale → Δ (명령이 0이면 보정 0) → 더하고 max_lin/max_ang로 자름.
    """
    episode_length_s = 20.0
    random_yaw = True
    action_space = 2               # [Δv, Δw] 정책 출력 [-1, 1] (노드가 × action_scale)
    observation_space = 6
    state_space = 0

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=512, env_spacing=3.0, replicate_physics=True)   # --num_envs
    viewer: ViewerCfg = ViewerCfg(eye=(2.0, -2.0, 1.5), lookat=(0.0, 0.0, 0.0), origin_type="env", env_index=0)   # 창: env 0 로봇 근처
    events: EventCfg | None = EventCfg()                    # env마다 바퀴 마찰 랜덤 — 끄려면 env.events=null
    ground: GroundCfg = GroundCfg()   # 배경 (ground.py): 평평 + 요철 · 마찰 패치 — env.ground.uneven=false · env.ground.friction_patches=0

    # 명령 (command.py) — 한 값을 hold 동안 유지
    command_hold = (0.5, 2.0)      # [s]
    t_change_max = 2.0             # [s] 관측 t_change 상한 (노드 t_change_max와 같게)

    # 정책 출력 [-1, 1] → 보정량 (노드 action_scale_lin/ang · max_delta와 같아야 한다)
    action_scale = (0.05, 0.10)    # [m/s, rad/s]

    # 에피소드마다 무작위 — 중심 = 로봇 실측 응답 (robot_cfg.py RESPONSE_*), 여기는 폭만
    tau_width = 0.10               # [s] τ = 실측 ± 이만큼 (최소 tau_min)
    tau_min = 0.01
    gain_range = (0.90, 1.10)      # 바퀴별 이득 — 실측 좌/우 gain에 곱함
    delay_width = 2                # 명령 지연 = 실측 ± 이만큼 [20 Hz 스텝] (최소 0)
    deadband_range = (0.0, 0.3)    # [rad/s] (실측 데드밴드 + 이 범위)
    obs_delay_steps = (0, 4)       # 센서 지연 (odom · imu)
    sep_nominal_err = 0.08         # 드라이버 기준 간격 vs 실제 유효 간격 (회전 오차 → 자이로로만 보임)
    radius_nominal_err = 0.03      # 기준 반지름 오차 (직진 거리)
    enc_noise = 0.005
    gyro_noise = 0.01
    gyro_bias = 0.005              # [rad/s] ±

    # 보상 — 몸체 속도가 명령을 따라가는 정도 (진짜 속도로 채점, 오차는 max_lin · max_ang로 나눠 단위 없앰)
    w_track_v = 10.0
    w_track_w = 10.0
    action_penalty = 0.01
    smooth_penalty = 0.05
