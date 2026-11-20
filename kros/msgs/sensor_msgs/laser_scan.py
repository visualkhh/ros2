from dataclasses import dataclass

from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class LaserScan:
    """ROS sensor_msgs/LaserScan 흉내 (scan_time, intensities는 생략). header.frame_id는 센서 좌표계(base_scan), stamp는 측정 시각.

    ranges[i]는 각도 angle_min + i * angle_increment (센서 앞쪽 0, 반시계 +)의 거리(m).
    범위 밖(range_min 미만/range_max 초과)은 ROS처럼 inf.
    """

    header: Header
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple[float, ...]
