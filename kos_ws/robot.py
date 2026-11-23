"""실기 TurtleBot3 Waffle Pi 개조형: OpenCR 보드(USB 시리얼) + LDS-03 라이다(USB2LDS).

시뮬레이터의 TurtleBot3와 같은 자리(부품 트리, command, joint_positions, lidar, imu)를 제공하므로
RobotBridge와 kros 노드들은 시뮬레이터인지 실기인지 모른다. 월드는 쓰지 않는다 (실시간 루프가 Clock을 내려준다).

- OpenCR: /dev/opencr (115200). 펌웨어 firmware/opencr/tb3_wheels. C 줄로 바퀴 속도, S/I/E 줄을 읽는다.
- 라이다: /dev/tb3_lidar 또는 /dev/ttyUSB0 (230400). LDS-03(COIN-D4) 패킷을 kros LaserScan으로 바꾼다.
"""
from pathlib import Path

from kros.hardwares.controllers.diff_drive_board import DiffDriveBoard
from kros.msgs.geometry_msgs import Twist
from kros.worlds.robots.robot import Robot
from kros.worlds.robots.turtlebot3.opencr_serial import OpenCrSerial
from kros.worlds.robots.turtlebot3.turtlebot3 import WHEEL_RADIUS, TurtleBot3

from kros.hardwares.lidars.lds_03 import SerialLds03

DESCRIPTION_DIR = Path(__file__).resolve().parent / "urdf"
URDF_PATH = DESCRIPTION_DIR / "turtlebot3_real.urdf"
OPENCR_PORT = "/dev/opencr"
LIDAR_MOUNT_X = 0.060  # m, base_link 기준 (실측, ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro)
LIDAR_MOUNT_HEIGHT = 0.022
WHEEL_SEPARATION = 0.287  # m, 실측 (Waffle Pi 개조형)
MAX_WHEEL = 7.8  # rad/s, 실기 최대 0.26 m/s


class RealTurtleBot3(TurtleBot3):
    """실기: 시뮬레이션 부품 대신 OpenCrSerial과 SerialLds03를 단다."""

    name = "turtlebot3_waffle_pi"
    description_dir = DESCRIPTION_DIR
    urdf_path = URDF_PATH

    def __init__(
        self,
        opencr_port: str = OPENCR_PORT,
        lidar_port: str = "",
        lidar_mount_x: float = LIDAR_MOUNT_X,
        lidar_mount_height: float = LIDAR_MOUNT_HEIGHT,
    ) -> None:
        Robot.__init__(self)  # 월드에 넣지 않는다: 실시간 루프가 tick을 준다
        self.opencr: DiffDriveBoard = OpenCrSerial(WHEEL_RADIUS, WHEEL_SEPARATION, opencr_port, max_wheel=MAX_WHEEL, left_dir=-1.0, right_dir=-1.0)
        self._lidar: SerialLds03 | None = SerialLds03(port=lidar_port, mount_x=lidar_mount_x, mount_height=lidar_mount_height, frame_id="base_scan")
        self.add(self.opencr, "opencr")
        if self._lidar is not None:
            self.add(self._lidar, "lidar")
        self.opencr.start()  # 포트가 없어도 실패하지 않고 1초마다 다시 연결한다

    @property
    def lidar(self) -> SerialLds03:
        if self._lidar is None:
            raise LookupError("이 로봇에는 라이다가 없다")
        return self._lidar

    def stop(self) -> None:
        """정지 명령을 보내고 시리얼을 닫는다 (C,0,0)."""
        try:
            self.opencr.write(Twist())
        finally:
            self.opencr.stop()
            if self._lidar is not None:
                self._lidar.stop()

    def __repr__(self) -> str:
        return f"<RealTurtleBot3 opencr={getattr(self.opencr, 'connected', False)} lidar={self.lidar.connected}>"
