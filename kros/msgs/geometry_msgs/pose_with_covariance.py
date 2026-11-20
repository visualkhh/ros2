from dataclasses import dataclass, field

from .pose import Pose


@dataclass(frozen=True)
class PoseWithCovariance:
    """ROS geometry_msgs/PoseWithCovariance 흉내. covariance는 (x, y, z, 회전 x, 회전 y, 회전 z) 6x6 행렬을 행 우선으로 편 36개 값."""

    pose: Pose = field(default_factory=Pose)
    covariance: tuple[float, ...] = (0.0,) * 36
