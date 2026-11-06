"""이 task의 로봇 설정. 모양 · 질량 · 관절은 usda/robot.usda (외부 제공 — 그대로 씀).

  숫자 (바퀴 · 모터 · 마찰)  표준 라이브러리만 — 리포트 · 실기기 측정 · dummy도 이 파일을 읽는다
  ROBOT_CFG                 Isaac Lab ArticulationCfg (ANYMAL_C_CFG 관례) — 처음 쓸 때 만든다 → usda/robot.usda

마찰 · 모터는 여기서 넣는다. USD 관절 이름 wheel_left_joint · wheel_right_joint.
숫자는 ROS 쪽과 같아야 한다 (바꾸면 양쪽 같이): ros2_ws/src/tb3_bringup/config/robot.yaml (바퀴) ·
opencr_driver.wheel_speeds (8 rad/s 비례 축소) · rl_nav_controller (max_lin/max_ang).
"""
from __future__ import annotations

import os

# 바퀴 · 속도 (드라이버 식)
WHEEL_RADIUS = 0.033   # [m]
WHEEL_SEP = 0.287      # [m] (드라이버 기준값)
MAX_LIN = 0.26         # [m/s]
MAX_ANG = 1.82         # [rad/s]
MAX_WHEEL = 8.0        # [rad/s] 펌웨어 VEL_LIMIT_RAD_S · 드라이버 max_wheel_speed
DIAG_RADIUS = 0.5      # [m] 텔레오퍼 사선 원호 (tool/cli/op/bus.py DIAG_RADIUS)

# 실측 바퀴 응답 — rl_correction_waypoint에서 복사 (같은 로봇 · 같은 USD). sim raw 대비 차이 (2026-11-03: isaac/artifacts/rl_correction_waypoint/calibration/2026-11-03_20-34-42_real_floor − …_20-22-20_sim_aa,
# real은 direct 경로). 이전 값(2026-11-02, direct): gain 0.9927/0.9938 · 지연 2.
# calibration으로 다시 재면 여기를 고친다. 학습 task는 이 값을 랜덤 범위의 중심으로 쓴다 (robots.use가 env cfg에 채움).
RESPONSE_GAIN_LR = (0.9937, 0.9931)   # 좌/우 바퀴 gain (real ÷ sim: 0.9947/1.0010 · 0.9943/1.0012)
RESPONSE_DELAY_STEPS = 3              # 명령 지연 [20 Hz 스텝] (real − sim: 3 − 0, real 0.173 s — PC 왕복 포함)
RESPONSE_TAU_S = 0.0                  # 1차 지연 τ [s] (real − sim: 0.0227 − 0.05 < 0 → 0)
RESPONSE_DEADBAND = 0.0               # [rad/s] (real 문턱 ÷ sim gain, 못 쟀으면 0)

# 바퀴 모터 — 고정 모델값 (실측 sweep해도 sim 응답이 안 변해서 calibration 변수가 아님)
WHEEL_DAMPING = 50.0         # 속도 서보 이득 (ImplicitActuator, stiffness 0)
WHEEL_EFFORT = 3.0           # [N·m] XM430-W210 stall 근사
WHEEL_VELOCITY_LIMIT = 9.0   # [rad/s] MAX_WHEEL(8) + 여유 — 포화는 env 바퀴 모델이 드라이버처럼 비례 축소로 직접

# 마찰 (static, dynamic) — env가 바퀴 · 캐스터 충돌체에 재질로 붙인다 (바닥 마찰은 env 쪽)
WHEEL_FRICTION = (1.0, 1.0)
CASTER_FRICTION = (0.0, 0.0)   # 캐스터는 min 결합 — average면 바닥 마찰에 끌려 바퀴가 덜 나감


def _robot_cfg():
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.from_files import UsdFileCfg

    return ArticulationCfg(
        prim_path="/World/Robot",
        spawn=UsdFileCfg(usd_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "usda", "robot.usda")),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.01)),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["wheel_left_joint", "wheel_right_joint"],
                effort_limit=WHEEL_EFFORT, velocity_limit=WHEEL_VELOCITY_LIMIT,
                stiffness=0.0, damping=WHEEL_DAMPING),
        },
    )


def __getattr__(name: str):
    """ROBOT_CFG는 처음 쓸 때 만든다 (import만으로는 isaaclab · USD 변환을 안 함)."""
    if name == "ROBOT_CFG":
        return globals().setdefault("ROBOT_CFG", _robot_cfg())
    raise AttributeError(name)
