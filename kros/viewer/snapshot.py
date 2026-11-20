from dataclasses import dataclass

from kros.msgs.geometry_msgs import Pose2D, Twist
from kros.msgs.sensor_msgs import LaserScan


@dataclass(frozen=True)
class Snapshot:
    """월드의 한 순간. 불변 순수 데이터라서 다른 스레드(나중엔 다른 프로세스)로 그냥 넘길 수 있다. 화면은 이것만 읽는다."""

    time: float  # 월드 시각(초)
    poses: tuple[Pose2D, ...]  # world.objects와 같은 순서의 위치
    robot_index: int  # poses에서 로봇의 자리
    command: Twist  # 로봇이 받은 속도 명령
    wheel_angles: tuple[float, float]  # 왼쪽, 오른쪽 바퀴 모터의 누적 회전각(rad). 화면의 바퀴 회전
    scan: LaserScan | None  # 로봇 라이다의 최신 측정 (새로 측정하기 전까지 같은 객체)
    scan_pose: Pose2D | None  # 그 스캔을 측정한 순간의 센서 위치
