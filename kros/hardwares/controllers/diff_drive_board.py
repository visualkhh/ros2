from abc import abstractmethod

from kros.hardwares.controllers.controller import Controller
from kros.hardwares.imus.imu import Imu
from kros.hardwares.motors.motor import Motor
from kros.msgs.geometry_msgs import Twist


class DiffDriveBoard(Controller):
    """바퀴 둘(차동 구동)과 IMU를 가진 제어 보드의 인터페이스: 몸체 속도 명령(Twist)을 받아 바퀴를 돌리고, 바퀴 위치/속도와 IMU를 내보낸다.
    시뮬레이션 보드(OpenCr)와 진짜 보드(OpenCrSerial)가 같은 모양이라 로봇은 어느 쪽이든 같은 방식으로 쓴다."""

    left: Motor
    right: Motor
    imu: Imu

    @property
    @abstractmethod
    def command(self) -> Twist:
        """가장 최근에 받은 몸체 속도 명령."""

    @abstractmethod
    def write(self, command: Twist) -> None:
        """몸체 속도 명령을 내린다 (객체째 교체라 다른 스레드에서 불러도 안전)."""
