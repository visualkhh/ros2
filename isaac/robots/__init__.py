"""로봇마다 폴더: robots/<robot>/{urdf/robot.urdf.xacro, meshes/, usd/(변환물, git 제외), config.py}.

  config.py     바퀴 숫자 · 모터 · 마찰 (표준 라이브러리로 읽힘) + ROBOT_CFG (Isaac Lab, 처음 쓸 때 만듦)
우리 로봇 = beebot (TurtleBot3 Waffle Pi 개조). 다른 로봇: `use(env_cfg, "<이름>" 또는 "<디렉토리>")` · ./run-world.sh --robot <이름|디렉토리>
"""
from __future__ import annotations

import importlib
import os

ROBOTS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT = "beebot"


def names() -> list[str]:
    return sorted(d for d in os.listdir(ROBOTS_DIR) if os.path.isfile(os.path.join(ROBOTS_DIR, d, "config.py")))


def robot_dir(robot: str) -> str:
    """이름(robots/<이름>) 또는 디렉토리 경로 → 로봇 폴더 절대경로. config.py가 없으면 SystemExit."""
    d = robot if (os.sep in robot or robot.startswith(".")) else os.path.join(ROBOTS_DIR, robot)
    d = os.path.abspath(d)
    if not os.path.isfile(os.path.join(d, "config.py")):
        raise SystemExit(f"로봇 없음: {robot} (config.py가 있는 폴더 — robots/에 있는 것: {', '.join(names())})")
    return d


def _module(path: str, name: str):
    import importlib.util

    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def use(env_cfg, robot: str) -> None:
    """env cfg의 로봇을 바꾼다 — robot = 이름(robots/<이름>) 또는 디렉토리 경로 (같은 모양: urdf/ · meshes/ · config.py).

    ROBOT_CFG(prim 경로는 env 것 유지) · 바퀴 숫자 · 바퀴/캐스터 마찰 — 전부 그 폴더의 config.py.
    """
    d = robot_dir(robot)
    if os.path.dirname(d) == ROBOTS_DIR:   # robots/ 안 = 패키지로
        cfg = importlib.import_module(f"robots.{os.path.basename(d)}.config")
    else:                                   # 밖 = 파일 경로로
        tag = "robot_" + os.path.basename(d)
        cfg = _module(os.path.join(d, "config.py"), tag + "_config")
    env_cfg.robot_cfg = cfg.ROBOT_CFG.replace(prim_path=env_cfg.robot_cfg.prim_path)
    env_cfg.wheel_radius, env_cfg.wheel_sep, env_cfg.max_wheel = cfg.WHEEL_RADIUS, cfg.WHEEL_SEP, cfg.MAX_WHEEL
    env_cfg.wheel_friction, env_cfg.caster_friction = cfg.WHEEL_FRICTION, cfg.CASTER_FRICTION
