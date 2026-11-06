"""이 task의 로봇 (beebot) 설정. 모양 · 질량 · 관절은 usda/robot.usda (Lv2_bee_Assignment isaac/sim/usda/beebot 복사 — 그대로 씀).

  숫자 (바퀴 · 모터 · 마찰)  표준 라이브러리만 — 리포트 · 실기기 측정 · dummy도 이 파일을 읽는다
  ROBOT_CFG                 Isaac Lab ArticulationCfg (ANYMAL_C_CFG 관례) — 처음 쓸 때 만든다 → usda/robot.usda

마찰 · 모터는 여기서 넣는다. USD 관절 이름 wheel_left_joint · wheel_right_joint.
숫자는 ROS 쪽과 같아야 한다 (바꾸면 양쪽 같이): Lv2_bee_Assignment ml.yaml rl_nav_correction (max_vel_x · max_vel_theta).
"""
from __future__ import annotations

import os

# 바퀴 · 속도 (드라이버 식) — beebot: 바퀴 반지름 · 간격은 Lv2_bee_Assignment ros_system controllers.yaml diff_drive_controller · 속도 한계는 ml rl_nav_correction(max_vel_x/theta)와 같다
WHEEL_RADIUS = 0.033   # [m]
WHEEL_SEP = 0.287      # [m] 실기기 diff_drive_controller wheel_separation (Lv2_bee_Assignment controllers.yaml)
MAX_LIN = 0.26         # [m/s]
MAX_ANG = 1.82         # [rad/s]
MAX_WHEEL = 8.0        # [rad/s] shortcut: beebot 펌웨어 한계는 확인 못 함 (Waffle Pi 값) — 다르면 여기를 고친다
DIAG_RADIUS = 0.5      # [m] 텔레오퍼 사선 원호

# 바퀴 응답 — 실측 (2026-11-06, rl_maze_beebot: sim 17:18:47 − real_floor 18:02:40)
# gain real÷sim 0.9979/0.9984 · 지연 1 step · τ 0.0636 s · deadband 0.0.
# ./run-calibration.sh rl_maze_beebot --robot sim|real 로 다시 재면 여기를 고친다. 학습은 이 값을 랜덤 범위의 중심으로 쓴다.
RESPONSE_GAIN_LR = (0.9979, 0.9984)   # 좌/우 바퀴 gain
RESPONSE_DELAY_STEPS = 1              # 명령 지연 [20 Hz 스텝] = 0.05 s
RESPONSE_TAU_S = 0.0636               # 1차 지연 τ [s]
RESPONSE_DEADBAND = 0.0               # [rad/s]

# 주행 중 팔 자세 = Lv2_bee_Assignment ros2_ws/src/skills/params/basket.yaml pose_stow (= ros_system controllers.yaml rest_pose · beebot.usda 기본 자세)
STOW = (0.0, -1.9024, 1.344, 0.9023, 0.0)   # [rad] joint1~5
GRIPPER_OPEN = 0.0112   # [m] 그 저장소 drive_env 주행 중 그리퍼 (간격 40 mm = GRIP_STOP 0.0062 + 5 mm) — shortcut: stow에 그리퍼 값은 따로 없어 이걸 씀

# 마찰 (static, dynamic) — env가 바퀴 · 캐스터 충돌체에 재질로 붙인다 (경로에 "wheel"/"caster"가 든 충돌체만 — beebot 캐스터는 sphere라 USD 마찰 그대로)
WHEEL_FRICTION = (1.0, 1.0)
CASTER_FRICTION = (0.0, 0.0)


def _robot_cfg():
    """beebot (바퀴 2 + 팔 5 + 그리퍼). 모터 · 관절 힘은 USD(physx 변형) 값 그대로 — Lv2_bee_Assignment place_env.robot_cfg와 같다.
    팔 · 그리퍼는 STOW로 붙잡아 둔다 (BaseEnv.hold_joints가 매 스텝 위치 목표를 준다)."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.from_files import UsdFileCfg
    from isaaclab.sim import ArticulationRootPropertiesCfg

    return ArticulationCfg(
        prim_path="/World/Robot",
        spawn=UsdFileCfg(usd_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "usda", "robot.usda"),
                         variants={"Physics": "physx"},
                         articulation_props=ArticulationRootPropertiesCfg(fix_root_link=False)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.01),
            joint_pos={**{f"joint{i + 1}": q for i, q in enumerate(STOW)}, "gripper_.*_joint": GRIPPER_OPEN}),
        actuators={
            "arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-5]"], stiffness=None, damping=None),
            "wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_.*_joint"], stiffness=None, damping=None),
            "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_.*_joint"], stiffness=None, damping=None),
        },
    )


def __getattr__(name: str):
    """ROBOT_CFG는 처음 쓸 때 만든다 (import만으로는 isaaclab · USD 변환을 안 함)."""
    if name == "ROBOT_CFG":
        return globals().setdefault("ROBOT_CFG", _robot_cfg())
    raise AttributeError(name)
