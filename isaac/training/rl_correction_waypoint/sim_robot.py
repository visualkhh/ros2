"""SimRobot — 이 로봇의 sim (Isaac). real_robot.py와 짝. 지금은 공용 robots/sim_robot.SimRobot 그대로.

sim 입출력을 이 로봇에 맞게 바꾸려면 여기서 덮어쓴다 (write = env.step([[v, w]]) · read = env.sense()).
"""
from __future__ import annotations

from robots.sim_robot import SimRobot as _SimRobot


class SimRobot(_SimRobot):
    pass
