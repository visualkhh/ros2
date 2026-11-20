from dataclasses import dataclass, field

from kros.msgs.geometry_msgs import Pose, Twist
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class Odometry:
    """ROS nav_msgs/Odometry 흉내 (공분산은 생략). pose는 header.frame_id(odom) 좌표계에서 본 child_frame_id(base_footprint)의 위치/방향이고,
    twist는 child_frame_id 축으로 본 속도(앞으로 linear.x, 회전 angular.z)다."""

    header: Header
    child_frame_id: str
    pose: Pose = field(default_factory=Pose)
    twist: Twist = field(default_factory=Twist)
