"""로봇마다 폴더: robots/<robot>/{urdf.xacro, meshes/, usd/(변환물, git 제외), spec.py(숫자), robot_cfg.py(ROBOT_CFG)}.

  spec.py       바퀴 반지름 · 간격 · 최대 속도 — 표준 라이브러리만 (report · 실기기 측정도 씀)
  robot_cfg.py  ROBOT_CFG (Isaac Lab ArticulationCfg: USD · 바퀴 모터) · 바퀴/캐스터 마찰 — AppLauncher 뒤에 import
우리 로봇 = beebot (TurtleBot3 Waffle Pi 개조). 다른 로봇: `use(env_cfg, "<이름>")` · ./run-world.sh --robot <이름>
"""
from __future__ import annotations

import importlib
import os

ROBOTS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT = "beebot"


def names() -> list[str]:
    return sorted(d for d in os.listdir(ROBOTS_DIR) if os.path.isfile(os.path.join(ROBOTS_DIR, d, "robot_cfg.py")))


def use(env_cfg, name: str) -> None:
    """env cfg의 로봇을 robots/<name>으로 — ROBOT_CFG(prim 경로는 env 것 유지) · 바퀴 숫자 · 바퀴/캐스터 마찰."""
    cfg = importlib.import_module(f"robots.{name}.robot_cfg")
    spec = importlib.import_module(f"robots.{name}.spec")
    env_cfg.robot_cfg = cfg.ROBOT_CFG.replace(prim_path=env_cfg.robot_cfg.prim_path)
    env_cfg.wheel_radius, env_cfg.wheel_sep, env_cfg.max_wheel = spec.WHEEL_RADIUS, spec.WHEEL_SEP, spec.MAX_WHEEL
    env_cfg.wheel_friction, env_cfg.caster_friction = cfg.WHEEL_FRICTION, cfg.CASTER_FRICTION
