from dataclasses import dataclass

from kros.msgs.std_msgs import Header

from .transform import Transform


@dataclass(frozen=True)
class TransformStamped:
    """ROS geometry_msgs/TransformStamped 흉내: 시각이 붙은 좌표계 변환. header.frame_id가 **부모**, child_frame_id가 **자식** 좌표계다."""

    header: Header
    child_frame_id: str
    transform: Transform
