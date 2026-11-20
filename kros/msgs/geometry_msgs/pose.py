from dataclasses import dataclass, field

from .point import Point
from .quaternion import Quaternion


@dataclass(frozen=True)
class Pose:
    """ROS geometry_msgs/Pose 흉내: 3D 위치 + 방향."""

    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)
