from dataclasses import dataclass


@dataclass(frozen=True)
class Clock:
    """월드가 매 step마다 모든 WorldObject에 주는 시계 신호: 월드 시각(초)과 이번에 진행한 시간(dt)."""

    time: float
    dt: float
