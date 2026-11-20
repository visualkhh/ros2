from dataclasses import dataclass

from kros.msgs.std_msgs import Header

from .point import Point


@dataclass(frozen=True)
class PointStamped:
    """ROS geometry_msgs/PointStamped 흉내: header.frame_id 좌표계에서 본 점."""

    header: Header
    point: Point
