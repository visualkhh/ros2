"""ROBOTIS OpenCR 1.0 (TurtleBot3의 제어 보드): 바퀴 모터(Dynamixel) 둘과 IMU(MPU-9250)가 이 보드에 붙어 있고, 차동 구동 계산도 보드가 한다."""
from kros.hardwares.controllers.diff_drive_board import DiffDriveBoard
from kros.hardwares.controllers.diff_drive import DiffDrive
from kros.hardwares.imus.imu import Imu
from kros.hardwares.motors.motor import Motor
from kros.msgs.geometry_msgs import Twist


class OpenCr(DiffDriveBoard):
    """OpenCR 보드: 구동부(drive: 모터 둘 + 차동 구동 계산)와 imu를 부품으로 가진다. 보드를 끄면 둘 다 꺼진다.

    부품(모터, IMU)은 받아서 단다: 시뮬레이션이든 더미든 진짜든 끼울 수 있다. 바퀴/몸체 치수는 로봇마다 다르므로 로봇이 넘긴다
    (진짜 보드는 펌웨어가 모델별 값을 가진다). 라이다는 보드가 아니라 라즈베리파이에 USB로 꽂혀 있으니 여기에 없다.
    """

    model = "OpenCR1.0"

    def __init__(
        self, left_motor: Motor, right_motor: Motor, imu: Imu, wheel_radius: float, wheel_separation: float, body_radius: float, period: float | None = None
    ) -> None:
        super().__init__(period)
        self.drive = DiffDrive(left_motor, right_motor, wheel_radius, wheel_separation, body_radius)
        self.left, self.right, self.imu = left_motor, right_motor, imu
        self.add(self.drive, "drive")  # 구동부가 먼저, IMU가 나중: 같은 신호에서 IMU는 움직인 뒤의 위치를 본다
        self.add(self.imu, "imu")

    @property
    def command(self) -> Twist:
        return self.drive.command

    def write(self, command: Twist) -> None:
        self.drive.write(command)
