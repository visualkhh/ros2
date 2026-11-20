from dataclasses import dataclass, field

from .twist import Twist


@dataclass(frozen=True)
class TwistWithCovariance:
    """ROS geometry_msgs/TwistWithCovariance 흉내. covariance는 (vx, vy, vz, wx, wy, wz) 6x6 행렬을 행 우선으로 편 36개 값."""

    twist: Twist = field(default_factory=Twist)
    covariance: tuple[float, ...] = (0.0,) * 36
