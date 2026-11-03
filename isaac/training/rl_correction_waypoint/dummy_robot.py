"""DummyRobot — 이 로봇의 가짜 (Isaac/ROS 없이 calibration 흐름 확인). sim_robot.py · real_robot.py와 짝. 표준 라이브러리만.

지금은 공용 robots/dummy_robot.DummyRobot 그대로 (바퀴 숫자 = 이 폴더 robot_cfg.py). 가짜 모터를 이 로봇에 맞게 바꾸려면 여기서.
"""
from __future__ import annotations

import os

from robots.dummy_robot import DummyRobot as _DummyRobot


class DummyRobot(_DummyRobot):
    def __init__(self, *args, robot: str = os.path.dirname(os.path.abspath(__file__)), **kw):
        super().__init__(*args, robot=robot, **kw)
