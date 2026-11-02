"""rl_correct v2 config — 우리 Waffle Pi(URDF)로 '실제로 관측되는 것만' 보정 (documents/isaac/task-spec.md §3)."""
from __future__ import annotations

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import UsdFileCfg
from isaaclab.utils import configclass

from urdf import robot_spec as spec
from calibration.parameter.ActuatorParam import ActuatorParam
from calibration.parameter.FrictionParam import FrictionParam

_FLOOR_FRICTION = FrictionParam()   # 기본값 (숨은 sim_calib_*.json은 안 읽는다)
_ACT = ActuatorParam()               # calibration sim과 같은 바퀴 모터 (fit 차이를 그대로 쓰려면 같아야 함)


@configclass
class RlCorrectEnvCfg(DirectRLEnvCfg):
    decimation = round(spec.PHYSICS_HZ / spec.CONTROL_HZ)  # 480 Hz 물리 -> 20 Hz 정책 (= cmd_manager)
    episode_length_s = 10.0
    action_space = 2       # [dV, dW]  (ONNX 출력은 action_scale 곱한 물리값)
    observation_space = 6  # [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / spec.PHYSICS_HZ, render_interval=decimation)

    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=UsdFileCfg(usd_path=spec.robot_usd()),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.01)),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["wheel_left_joint", "wheel_right_joint"],
                effort_limit=_ACT.effort,
                velocity_limit=_ACT.velocity_limit,    # 포화는 환경이 비례 축소로 직접 (드라이버와 같게)
                stiffness=0.0,
                damping=_ACT.damping,                  # 속도 서보 — 고정 모델값 (ActuatorParam)
            ),
        },
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=2048, env_spacing=3.0, replicate_physics=True)

    # 정책 출력 [-1, 1] → 보정량 (ONNX export도 같은 값으로 곱해 물리값을 낸다)
    action_scale = (0.10, 0.60)  # [m/s, rad/s]

    # 에피소드마다 무작위 (실제에서 엔코더·자이로로 보이는 오차만)
    tau_range = (0.05, 0.25)       # [s] 바퀴별 1차 지연
    gain_range = (0.90, 1.10)      # 바퀴별 이득 (좌우 비대칭)
    gain_center = (1.0, 1.0)       # 좌/우 gain 중심 — gain_range 샘플에 곱함 (`--fit`이면 real fit)
    deadband_range = (0.0, 0.3)    # [rad/s] 이보다 작은 바퀴 명령은 안 돎
    # documents/isaac/task-spec.md §3 기준: 명령·관측 각각 0~4 스텝 (0~200 ms)
    cmd_delay_steps = (0, 4)       # PC → Pi (×50 ms)
    obs_delay_steps = (0, 4)       # Pi → PC (odom·imu)
    sep_nominal_err = 0.08         # 드라이버 기준 간격 vs 실제 유효 간격 (회전 오차 → 자이로로만 보임)
    radius_nominal_err = 0.03      # 기준 반지름 오차 (직진 거리 — 관측 안 됨, 강건성용)
    enc_noise = 0.005
    gyro_noise = 0.01

    # 바닥/바퀴 마찰 — 학습은 기본으로 env별 랜덤+패치 (train.py가 켬), `--no-friction`이면 FrictionParam 기본값 고정
    ground_friction = _FLOOR_FRICTION.ground
    wheel_friction = _FLOOR_FRICTION.wheel
    caster_friction = _FLOOR_FRICTION.caster
    randomize_friction = False
    ground_static_range = (0.50, 0.90)
    ground_dynamic_range = (0.40, 0.80)
    # env 안 요철 (울퉁불퉁) — 바퀴가 올라타는 낮은 턱. 기하학적 외란. `--uneven`으로 켠다.
    randomize_uneven = False
    uneven_patch_count = 4
    uneven_patch_size = (0.30, 0.55)    # [m] 한 변
    uneven_patch_height = (0.01, 0.03)  # [m] 높이 — "약간"
    # env 바닥 기울기 [deg] — env마다 방향 랜덤 · 0~slope_deg. 0이면 평평. `--slope`로 켠다.
    # 오르막/내리막 = 바퀴 부하로 속도 처짐, 옆 기울기 = 아래로 흘러 휘어짐 (실내 바닥 1~2°, 경사로 ~5°)
    slope_deg = 0.0
    # env 안의 국소 마찰 패치 (파랑=미끄럼, 주황=끈적)
    friction_patch_count = 4
    friction_patch_radius = (0.55, 0.90)
    friction_patch_size = (0.35, 0.55)
    slippery_friction = (0.20, 0.15)
    sticky_friction = (1.10, 0.90)

    # 명령: hold_range 초마다 바뀜 (텔레오퍼 키 · 부드러운 곡선 · 정지)
    hold_range = (0.5, 2.0)
    p_teleop, p_smooth = 0.6, 0.3   # 나머지 = 정지

    # 보상
    ang_weight = 0.5
    action_penalty = 0.01
    smooth_penalty = 0.05
