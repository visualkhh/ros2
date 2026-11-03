"""beebot — TurtleBot3 Waffle Pi 개조 (상판 제거 · LDS-03 1층 앞). 로봇 설정 한 파일.

  숫자 (바퀴 · 모터 · 마찰)  표준 라이브러리만 — 리포트 · 실기기 측정 · dummy도 이 파일을 읽는다
  ROBOT_CFG                 Isaac Lab ArticulationCfg (ANYMAL_C_CFG 관례) — 처음 쓸 때 만든다 (USD 경로만, 변환은 env가)

모양 · 질량 · 관절은 urdf/robot.urdf.xacro → usd/ (robots.urdf_to_usd). 마찰 · 모터는 URDF에 없어서 여기서 넣는다.
숫자는 ROS 쪽과 같아야 한다 (바꾸면 양쪽 같이): ros2_ws/src/tb3_bringup/config/robot.yaml (바퀴) ·
opencr_driver.wheel_speeds (8 rad/s 비례 축소) · rl_correction corrector (max_lin/max_ang).
"""
from __future__ import annotations

import os

# 바퀴 · 속도 (드라이버 식)
WHEEL_RADIUS = 0.033   # [m]
WHEEL_SEP = 0.287      # [m] (드라이버 기준값. URDF 바퀴 y ±0.144)
MAX_LIN = 0.26         # [m/s]
MAX_ANG = 1.82         # [rad/s]
MAX_WHEEL = 8.0        # [rad/s] 펌웨어 VEL_LIMIT_RAD_S · 드라이버 max_wheel_speed
DIAG_RADIUS = 0.5      # [m] 텔레오퍼 사선 원호 (tool/cli/op/bus.py DIAG_RADIUS)

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

    from robots.urdf_to_usd import usd_path

    return ArticulationCfg(
        prim_path="/World/Robot",
        spawn=UsdFileCfg(usd_path=usd_path(os.path.dirname(os.path.abspath(__file__)))),   # 이 폴더 (변환은 env가 장면 만들 때)
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
    if name in ("ROBOT_CFG", "BEEBOT_CFG"):
        cfg = globals().setdefault("ROBOT_CFG", _robot_cfg())
        globals()["BEEBOT_CFG"] = cfg
        return cfg
    raise AttributeError(name)
