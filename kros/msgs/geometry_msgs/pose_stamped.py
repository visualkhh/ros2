from dataclasses import dataclass

from kros.msgs.std_msgs import Header

from .pose import Pose


@dataclass(frozen=True)
class PoseStamped:
    """ROS geometry_msgs/PoseStamped 흉내: header.frame_id 좌표계에서 본 자세."""

    header: Header
    pose: Pose
