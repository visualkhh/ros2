from dataclasses import dataclass, field

from .vector3 import Vector3


@dataclass(frozen=True)
class Twist:
    """ROS geometry_msgs/Twist 흉내: 선속도 + 각속도."""

    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)
