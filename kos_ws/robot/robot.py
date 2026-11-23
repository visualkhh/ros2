"""이 프로젝트(kos_ws)의 실기 TurtleBot3 Waffle Pi 개조형.

kros의 공용 부품/노드는 그대로 쓰고, **이 로봇에만 해당하는 것**(치수, 장착 위치, OpenCR 시리얼, URDF/메시)은
kos_ws가 소유한다. 나중에 이 파일과 hardware/, description/을 고치면 kros(공용 라이브러리)는 건드리지 않는다.

- OpenCR: /dev/opencr (115200). 펌웨어 firmware/opencr/tb3_wheels. C 줄로 바퀴 속도, S/I/E 줄을 읽는다.
- 라이다: /dev/tb3_lidar 또는 /dev/ttyUSB0 (230400). LDS-03(COIN-D4) 패킷을 kros LaserScan으로 바꾼다.
- 월드는 쓰지 않는다: 실시간 루프가 Clock을 내려준다 (main.py).
"""
from pathlib import Path

from kros.msgs.geometry_msgs import Twist
from kros.worlds.robots.robot import Robot

from kos_ws.hardwares.lds_03 import SerialLds03
from kos_ws.hardwares.opencr import OpenCrSerial
from .turtlebot3 import TurtleBot3

DESCRIPTION_DIR = Path(__file__).resolve().parent / "description"  # package://turtlebot3_description/ 에 해당
URDF_PATH = DESCRIPTION_DIR / "urdf" / "turtlebot3_real.urdf"
OPENCR_PORT = "/dev/opencr"
LIDAR_MOUNT_X = 0.060  # m, base_link 기준 (실측, ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro)
LIDAR_MOUNT_HEIGHT = 0.022  # m, 스캔 평면 높이 (1층 판 위)
WHEEL_RADIUS = 0.033  # m (Waffle Pi)
WHEEL_SEPARATION = 0.287  # m, 두 바퀴 중심 사이 (실측)
MAX_WHEEL = 7.8  # rad/s, 실기 최대 0.26 m/s


class RealTurtleBot3(TurtleBot3):
    """실기: OpenCR 보드(바퀴+IMU)와 LDS-03 라이다를 단다. kos_ws/hardwares의 로봇 정의를 상속한다."""

    name = "turtlebot3_waffle_pi"
    description_dir = DESCRIPTION_DIR
    urdf_path = URDF_PATH
    radius = 0.22  # m, 충돌/코스트맵 반지름 (실기 몸통을 덮음)

    def __init__(
        self,
        opencr_port: str = OPENCR_PORT,
        lidar_port: str = "",
        lidar_mount_x: float = LIDAR_MOUNT_X,
        lidar_mount_height: float = LIDAR_MOUNT_HEIGHT,
    ) -> None:
        Robot.__init__(self)  # 월드에 넣지 않는다: 실시간 루프가 tick을 준다
        # 이 로봇은 두 모터 모두 +속도 = 뒤로 (ROS 쪽 robot.yaml left_dir/right_dir = -1).
        self.opencr: OpenCrSerial = OpenCrSerial(
            WHEEL_RADIUS,
            WHEEL_SEPARATION,
            opencr_port,
            max_wheel=MAX_WHEEL,
            left_dir=-1.0,
            right_dir=-1.0,
        )
        self._lidar: SerialLds03 | None = SerialLds03(
            port=lidar_port,
            mount_x=lidar_mount_x,
            mount_height=lidar_mount_height,
            frame_id="base_scan",
            period=0.05,  # 20Hz로 확인: 라이다 스캔(10Hz)을 놓치지 않는다
            reverse=True,  # 실측: 이 LDS-03은 각도가 시계방향이라 그대로 쓰면 좌우가 뒤집힌다
        )
        self.add(self.opencr, "opencr")
        if self._lidar is not None:
            self.add(self._lidar, "lidar")
        self.opencr.start()  # 포트가 없어도 실패하지 않고 1초마다 다시 연결한다

    def stop(self) -> None:
        """정지 명령(C,0,0)을 보내고 시리얼을 닫는다."""
        try:
            self.opencr.write(Twist())
        finally:
            self.opencr.stop()
            if self._lidar is not None:
                self._lidar.stop()

    def __repr__(self) -> str:
        return f"<RealTurtleBot3 opencr={self.opencr.connected} lidar={self._lidar.connected if self._lidar is not None else False}>"
