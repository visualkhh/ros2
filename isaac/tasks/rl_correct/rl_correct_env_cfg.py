"""rl_correct v2 config — 우리 Waffle Pi(URDF)로 '실제로 관측되는 것만' 보정 (documents/isaac/task-spec.md §3)."""
from __future__ import annotations

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import UsdFileCfg
from isaaclab.utils import configclass

from tasks.common import robot_spec as spec


@configclass
class RlCorrectEnvCfg(DirectRLEnvCfg):
    decimation = round(120 / spec.CONTROL_HZ)  # 120 Hz sim -> 20 Hz policy (= cmd_manager)
    episode_length_s = 10.0
    action_space = 2       # [dV, dW]  (ONNX 출력은 action_scale 곱한 물리값)
    observation_space = 6  # [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=UsdFileCfg(usd_path=spec.robot_usd()),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.01)),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["wheel_left_joint", "wheel_right_joint"],
                effort_limit=3.0,                      # XM430-W210 stall ~3 N·m
                velocity_limit=spec.MAX_WHEEL + 1.0,   # 포화는 환경이 비례 축소로 직접 (드라이버와 같게)
                stiffness=0.0,
                damping=20.0,                          # 속도 서보 (Dynamixel 속도 모드처럼 정상상태 오차 ≈ 0) — 지연은 환경의 1차 지연 모델이 담당
            ),
        },
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=2048, env_spacing=3.0, replicate_physics=True)

    # 정책 출력 [-1, 1] → 보정량 (ONNX export도 같은 값으로 곱해 물리값을 낸다)
    action_scale = (0.10, 0.60)  # [m/s, rad/s]

    # 에피소드마다 무작위 (실제에서 엔코더·자이로로 보이는 오차만)
    tau_range = (0.05, 0.25)       # [s] 바퀴별 1차 지연
    gain_range = (0.90, 1.10)      # 바퀴별 이득 (좌우 비대칭)
    deadband_range = (0.0, 0.3)    # [rad/s] 이보다 작은 바퀴 명령은 안 돎
    cmd_delay_steps = (0, 2)       # PC → Pi (×50 ms)
    obs_delay_steps = (0, 2)       # Pi → PC (odom·imu)
    sep_nominal_err = 0.08         # 드라이버 기준 간격 vs 실제 유효 간격 (회전 오차 → 자이로로만 보임)
    radius_nominal_err = 0.03      # 기준 반지름 오차 (직진 거리 — 관측 안 됨, 강건성용)
    enc_noise = 0.005
    gyro_noise = 0.01

    # 명령: hold_range 초마다 바뀜 (텔레오퍼 키 · 부드러운 곡선 · 정지)
    hold_range = (0.5, 2.0)
    p_teleop, p_smooth = 0.6, 0.3   # 나머지 = 정지

    # 보상
    ang_weight = 0.5
    action_penalty = 0.01
    smooth_penalty = 0.05
