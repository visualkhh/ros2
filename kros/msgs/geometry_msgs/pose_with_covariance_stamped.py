from dataclasses import dataclass, field

from kros.msgs.std_msgs import Header

from .pose_with_covariance import PoseWithCovariance


@dataclass(frozen=True)
class PoseWithCovarianceStamped:
    """ROS geometry_msgs/PoseWithCovarianceStamped 흉내: header.frame_id 좌표계의 위치와 그 불확실성 (rviz 2D Pose Estimate가 /initialpose로 낸다)."""

    header: Header
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
