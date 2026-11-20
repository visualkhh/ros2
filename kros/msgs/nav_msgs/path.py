from dataclasses import dataclass

from kros.msgs.geometry_msgs import PoseStamped
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class Path:
    """ROS nav_msgs/Path 흉내: 따라갈 위치들 (header.frame_id 좌표계, 보통 map)."""

    header: Header
    poses: tuple[PoseStamped, ...] = ()
