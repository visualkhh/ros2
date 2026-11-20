from dataclasses import dataclass


@dataclass(frozen=True)
class Point:
    """ROS geometry_msgs/Point 흉내: 3D 위치."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
