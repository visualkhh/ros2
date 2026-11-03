"""RobotInterface — sim · real · 가짜 로봇을 같은 방식으로: write(v, w)로 명령을 보내고 read()로 상태 한 줄을 읽는다.

ros2_control hardware_interface의 read() · write()와 같은 역할. 명령(BaseCommand)은 모른다 — 그건 sysid.record()가 준다.
  SimInterface (sim_interface.py)       env.step([[v, w]])        — Isaac
  RealInterface (real_interface.py)     /cmd_manual_vel publish   — ROS
  DummyInterface (dummy_interface.py)   가짜 모터 계산            — 표준 라이브러리만
이 파일은 표준 라이브러리만.
"""
from __future__ import annotations


class RobotInterface:
    tag = "robot"

    def __init__(self, dt: float):
        self.dt = dt   # 한 스텝 [s] — write · read 주기

    def start(self) -> None:
        """기록 시작 직전 (real: 벽시계 기준점)."""

    def write(self, v: float, w: float) -> None:
        """명령 (v [m/s], w [rad/s])을 보낸다."""
        raise NotImplementedError

    def pace(self, k: int) -> None:
        """k번째 스텝이 끝날 때까지 기다림 — real만 (sim은 물리 스텝이 시계, dummy는 계산만)."""

    def read(self, t: float, v: float, w: float) -> list:
        """지금 상태 → CSV 한 줄 (sysid.CSV_HEADER 순서). v, w = 방금 보낸 명령."""
        raise NotImplementedError
