"""beebot Isaac 설정 — ROBOT_CFG (Isaac Lab ArticulationCfg) · 바퀴 모터 · 바퀴/캐스터 마찰.

Isaac Lab `isaaclab_assets/robots/anymal.py`의 `ANYMAL_C_CFG`와 같은 관례: env cfg가 `ROBOT_CFG.replace(prim_path=...)`.
모양 · 질량 · 관절은 urdf/robot.urdf.xacro → usd/ (robots.urdf_to_usd.robot_usd). 마찰 · 모터는 URDF에 없어서 여기서 넣는다.
USD 변환에 Isaac 앱이 필요 — AppLauncher 뒤에 import.
"""
from __future__ import annotations

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sim.spawners.from_files import UsdFileCfg

from robots.urdf_to_usd import robot_usd

# 바퀴 모터 — 고정 모델값 (실측 sweep해도 sim 응답이 안 변해서 calibration 변수가 아님)
WHEEL_DAMPING = 50.0         # 속도 서보 이득 (ImplicitActuator, stiffness 0)
WHEEL_EFFORT = 3.0           # [N·m] XM430-W210 stall 근사
WHEEL_VELOCITY_LIMIT = 9.0   # [rad/s] MAX_WHEEL(8) + 여유 — 포화는 env 바퀴 모델이 드라이버처럼 비례 축소로 직접

# 마찰 (static, dynamic) — env가 바퀴 · 캐스터 충돌체에 재질로 붙인다 (바닥 마찰은 env 쪽)
WHEEL_FRICTION = (1.0, 1.0)
CASTER_FRICTION = (0.0, 0.0)   # 캐스터는 min 결합 — average면 바닥 마찰에 끌려 바퀴가 덜 나감

ROBOT_CFG = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=UsdFileCfg(usd_path=robot_usd(os.path.dirname(os.path.abspath(__file__)))),   # 이 폴더 — 복사해 가도 그대로
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.01)),
    actuators={
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["wheel_left_joint", "wheel_right_joint"],
            effort_limit=WHEEL_EFFORT, velocity_limit=WHEEL_VELOCITY_LIMIT,
            stiffness=0.0, damping=WHEEL_DAMPING),
    },
)
BEEBOT_CFG = ROBOT_CFG
