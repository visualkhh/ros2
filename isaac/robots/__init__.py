"""로봇 폴더 = robot_cfg.py + USD. 정해진 모양 두 가지:

  training/<task>/{robot_cfg.py, usda/robot.usda}       외부에서 받은 USD 그대로 (변환 없음) — 기본 로봇
  robots/<이름>/{robot_cfg.py, urdf/robot.urdf.xacro, meshes/}  URDF → usd/<이름>/<이름>.usda 변환 (robots.urdf_to_usd)
  robot_cfg.py  바퀴 숫자 · 모터 · 마찰 · 실측 응답 (표준 라이브러리로 읽힘) + ROBOT_CFG (Isaac Lab, 처음 쓸 때 만듦)
  real_robot.py RealRobot — 그 로봇의 실기기 ROS 입출력 (robots.load(robot, "real_robot"))
입출력 공통: robot_interface.RobotInterface (write · read) — sim_robot.SimRobot · dummy_robot.DummyRobot · <로봇>/real_robot.RealRobot
다른 로봇: `use(env_cfg, "<이름>" 또는 "<디렉토리>")` · ./run-world.sh --robot <이름|디렉토리>
"""
from __future__ import annotations

import os

ROBOTS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT = os.path.join(os.path.dirname(ROBOTS_DIR), "training", "rl_correction_waypoint")   # 기본 로봇 = 이 task의 로봇
SETTINGS = "robot_cfg.py"


def names() -> list[str]:
    return sorted(d for d in os.listdir(ROBOTS_DIR) if os.path.isfile(os.path.join(ROBOTS_DIR, d, SETTINGS)))


def robot_dir(robot: str) -> str:
    """이름(robots/<이름>) 또는 디렉토리 경로 → 로봇 폴더 절대경로. robot_cfg.py가 없으면 SystemExit."""
    d = robot if (os.sep in robot or robot.startswith(".")) else os.path.join(ROBOTS_DIR, robot)
    d = os.path.abspath(d)
    if not os.path.isfile(os.path.join(d, SETTINGS)):
        raise SystemExit(f"로봇 없음: {robot} ({SETTINGS}가 있는 폴더 — 기본 {DEFAULT} · robots/에 있는 것: {', '.join(names()) or '없음'})")
    return d


def load(robot: str = DEFAULT, name: str = "robot_cfg"):
    """로봇 폴더의 <name>.py 모듈 — robot_cfg(바퀴 숫자 등, 표준 라이브러리만) · real_robot(RealRobot). 같은 파일은 한 번만 읽는다."""
    import importlib.util
    import sys

    path = os.path.join(robot_dir(robot), name + ".py")
    if not os.path.isfile(path):
        raise SystemExit(f"로봇 폴더에 {name}.py 없음: {path}")
    name = "robot_cfg_" + path.replace(os.sep, "_").replace(".", "_")
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, path)
        sys.modules[name] = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sys.modules[name])
    return sys.modules[name]


def use(env_cfg, robot: str) -> None:
    """env cfg의 로봇을 바꾼다 — robot = 이름(robots/<이름>) 또는 디렉토리 경로 (robot_cfg.py가 있는 폴더).

    ROBOT_CFG(prim 경로는 env 것 유지) · 바퀴 숫자 · 실측 응답 · 바퀴/캐스터 마찰 — 전부 그 폴더의 robot_cfg.py.
    """
    cfg = load(robot)
    env_cfg.robot_cfg = cfg.ROBOT_CFG.replace(prim_path=env_cfg.robot_prim_path)
    env_cfg.wheel_radius, env_cfg.wheel_sep, env_cfg.max_wheel = cfg.WHEEL_RADIUS, cfg.WHEEL_SEP, cfg.MAX_WHEEL
    env_cfg.max_lin, env_cfg.max_ang, env_cfg.diag_radius = cfg.MAX_LIN, cfg.MAX_ANG, cfg.DIAG_RADIUS
    env_cfg.response_gain_lr, env_cfg.response_delay_steps = cfg.RESPONSE_GAIN_LR, cfg.RESPONSE_DELAY_STEPS
    env_cfg.response_tau_s, env_cfg.response_deadband = cfg.RESPONSE_TAU_S, cfg.RESPONSE_DEADBAND
    env_cfg.wheel_friction, env_cfg.caster_friction = cfg.WHEEL_FRICTION, cfg.CASTER_FRICTION
