import math

from kros.msgs.geometry_msgs import Pose2D
from kros.msgs.sensor_msgs import LaserScan
from kros.worlds.hardwares.hardware import Sensor


class Lidar(Sensor[LaserScan]):
    """2D 라이다 인터페이스: 주기마다 LaserScan을 내보낸다. read()/subscribe()로 받는다.
    스캔을 만드는 sample()은 모델/구현(시뮬레이션, 더미, 진짜)이 각자 한다.

    어디에 달렸는가(mount_x: 로봇 중심에서 앞쪽으로 m, mount_height: 바닥에서 센서까지 m)는 부품이 아니라 로봇이 정해 넘긴다.
    """

    def __init__(self, mount_x: float = 0.0, mount_height: float = 0.0, period: float | None = None) -> None:
        super().__init__(period)
        self.mount_x = mount_x
        self.mount_height = mount_height
        self.scan_pose: Pose2D | None = None  # 최신 스캔을 측정한 순간의 센서 위치 (화면 표시용)

    def sensor_pose(self, robot: Pose2D) -> Pose2D:
        """로봇 위치에서 센서가 있는 위치."""
        c, s = math.cos(robot.theta), math.sin(robot.theta)
        return Pose2D(robot.x + self.mount_x * c, robot.y + self.mount_x * s, robot.theta)
