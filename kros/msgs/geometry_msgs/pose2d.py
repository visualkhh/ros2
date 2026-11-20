from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """평면 위치(m)와 방향(rad, 0 = +x 방향, 반시계가 +)."""

    x: float
    y: float
    theta: float
