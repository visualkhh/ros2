from dataclasses import dataclass, field

from .quaternion import Quaternion
from .vector3 import Vector3


@dataclass(frozen=True)
class Transform:
    """ROS geometry_msgs/Transform 흉내: 3D 이동(translation) + 회전(rotation). 자식 좌표계의 점을 부모 좌표계로 옮긴다."""

    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)
