from dataclasses import dataclass


@dataclass(frozen=True)
class Quaternion:
    """ROS geometry_msgs/Quaternion 흉내: 3D 회전. 기본은 회전 없음 (0, 0, 0, 1)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0
