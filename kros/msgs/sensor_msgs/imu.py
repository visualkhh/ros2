from dataclasses import dataclass, field

from kros.msgs.geometry_msgs import Quaternion, Vector3
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class Imu:
    """ROS sensor_msgs/Imu 흉내. header.frame_id는 센서 좌표계(imu_link).

    orientation: 월드에서 본 센서의 방향, angular_velocity: 센서 축 기준 각속도 (rad/s),
    linear_acceleration: 센서 축 기준 가속도 (m/s^2, 가속도계가 재는 값이라 가만히 있어도 중력 때문에 위쪽(+z)으로 g가 나온다).
    *_covariance: 각 값의 3x3 공분산을 행 우선으로 편 9개 값. ROS처럼 전부 0이면 "모름", 첫 값이 -1이면 "그 값이 없음"이다.
    """

    header: Header
    orientation: Quaternion = field(default_factory=Quaternion)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    linear_acceleration: Vector3 = field(default_factory=Vector3)
    orientation_covariance: tuple[float, ...] = (0.0,) * 9
    angular_velocity_covariance: tuple[float, ...] = (0.0,) * 9
    linear_acceleration_covariance: tuple[float, ...] = (0.0,) * 9
