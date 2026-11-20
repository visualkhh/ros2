from pathlib import Path
from typing import Any

from kros.msgs.geometry_msgs import Pose2D, Twist, Vector3
from kros.hardwares import Imu, Lidar
from kros.hardwares.controllers.diff_drive_board import DiffDriveBoard
from kros.hardwares.controllers.diff_drive import DiffDrive
from kros.hardwares.controllers.opencr import OpenCr
from kros.hardwares.imus.mpu9250 import SimulatedMpu9250
from kros.hardwares.lidars.lds_01 import SimulatedLds01
from kros.hardwares.motors.xl430_w250 import SimulatedXl430W250
from kros.interactions import Interaction, KeyState
from kros.worlds.robots.robot import Robot

from .opencr_serial import OpenCrSerial


DESCRIPTION_DIR = Path(__file__).parent / "description"  # package://turtlebot3_description/ 에 해당
LIDAR_MOUNT_X = -0.064  # m, base_footprint 기준 라이다 위치 (URDF scan_joint)
LIDAR_MOUNT_HEIGHT = 0.132  # m, 바닥에서 라이다까지 (URDF: 0.010 + 0.122)
WHEEL_RADIUS = 0.033  # m (URDF)
WHEEL_SEPARATION = 0.288  # m, 두 바퀴 사이 (URDF: 바퀴 y = ±0.144)
LINEAR_SPEED = 0.26  # m/s, waffle pi 최대 (키보드 조종 기본 속도)
ANGULAR_SPEED = 1.5  # rad/s
SPEED_STEP = 1.25  # q/z를 한 번 누를 때 곱하는 배율
SCALE_MIN, SCALE_MAX = 0.25, 4.0  # 기본 속도 대비. 4배면 실제 로봇 한계를 넘는 시뮬레이션 전용 값


class TurtleBot3(Robot):
    """ROBOTIS TurtleBot3 waffle pi: 구동부(바퀴 모터 둘), 라이다, IMU를 단 로봇. 키보드를 receive()하면 키로 조종한다."""

    name = "turtlebot3_waffle_pi"
    description_dir = DESCRIPTION_DIR
    urdf_path = DESCRIPTION_DIR / "urdf" / "turtlebot3_waffle_pi.urdf"
    mass = 1.8  # kg, 실제 waffle pi. movable은 False: 모터가 스스로 움직이고 부딪혀도 밀리지 않는다
    height = 0.141  # m, waffle pi 높이 (라이다 높이 0.132보다 높아서 다른 로봇의 라이다에 보인다)
    radius = 0.2  # waffle pi를 바퀴축 중심 원으로 근사 (뒤쪽 끝 0.197). 회전 중 뒤 모서리(0.25)는 벽을 살짝 파고들 수 있음

    def __init__(self, pose: Pose2D = Pose2D(0.0, 0.0, 0.0), serial_port: str | None = None) -> None:
        """부품은 로봇이 정한다. 시뮬레이션이 기본이다: OpenCR 보드(XL430-W250 바퀴 모터 둘, MPU-9250)와 LDS-01 라이다.
        serial_port(예: "/dev/opencr")를 주면 진짜 OpenCR 보드를 시리얼로 쓴다 (진짜 라이다 구현은 아직 없어 라이다 없이 시작한다).
        부품을 바꾸려면(더미 등) make_opencr / make_lidar를 하위 클래스에서 덮어쓴다. 라이다는 라즈베리파이에 꽂히므로 보드가 아니라 로봇에 직접 달린다."""
        super().__init__()
        self.pose = pose
        self.opencr: DiffDriveBoard = self.make_opencr(serial_port)
        self._lidar: Lidar | None = self.make_lidar(serial_port)
        self.speed_scale = 1.0  # q/z로 바꾸는 키보드 조종 속도 배율
        self._keys: frozenset[str] = frozenset()  # 직전에 눌려 있던 키
        self.add(self.opencr, "opencr")  # 부품을 단다: 시계 신호가 Robot.tick -> OpenCR 보드(-> 구동부와 바퀴 모터들, IMU), 라이다 순으로 내려간다
        if self._lidar is not None:
            self.add(self._lidar, "lidar")

    def make_opencr(self, serial_port: str | None) -> DiffDriveBoard:
        """OpenCR 보드를 만든다: 시리얼 포트가 있으면 진짜, 없으면 시뮬레이션 (이 로봇의 바퀴/몸체 치수로)."""
        if serial_port is not None:
            return OpenCrSerial(WHEEL_RADIUS, WHEEL_SEPARATION, serial_port)
        return OpenCr(SimulatedXl430W250(), SimulatedXl430W250(), SimulatedMpu9250(), WHEEL_RADIUS, WHEEL_SEPARATION, self.radius)

    def make_lidar(self, serial_port: str | None) -> Lidar | None:
        """라이다를 만든다: 시뮬레이션은 LDS-01. 진짜는 아직 없다(None)."""
        return None if serial_port is not None else SimulatedLds01(LIDAR_MOUNT_X, LIDAR_MOUNT_HEIGHT)

    @property
    def has_lidar(self) -> bool:
        return self._lidar is not None

    @property
    def lidar(self) -> Lidar:
        if self._lidar is None:
            raise LookupError("이 로봇에는 라이다가 없다 (has_lidar로 먼저 확인한다)")
        return self._lidar

    @property
    def drive(self) -> DiffDrive:
        """시뮬레이션 보드의 구동부. 진짜 보드에는 없다."""
        if not isinstance(self.opencr, OpenCr):
            raise LookupError("구동부는 시뮬레이션 보드에만 있다")
        return self.opencr.drive

    @property
    def imu(self) -> Imu:
        return self.opencr.imu

    def joint_positions(self) -> dict[str, float]:
        """URDF의 관절 이름으로 바퀴 모터의 누적 회전각 (엔코더)."""
        return {"wheel_left_joint": self.opencr.left.position, "wheel_right_joint": self.opencr.right.position}

    def on_interaction(self, interaction: Interaction[Any], state: Any) -> None:
        """키보드 입력이 오면 키를 속도 명령으로 해석해 구동부에 내린다. w/s(up/down) 전후진, a/d(left/right) 좌/우회전,
        반대 키는 서로 상쇄, q/z는 속도 배율 올리기/내리기(누를 때마다 한 번)."""
        if not isinstance(state, KeyState):
            return
        keys = state.keys
        pressed = keys - self._keys  # 이번에 새로 눌린 키 (누르고 있는 동안 반복 증가 방지)
        self._keys = keys
        if "q" in pressed:
            self.speed_scale = min(SCALE_MAX, self.speed_scale * SPEED_STEP)
        if "z" in pressed:
            self.speed_scale = max(SCALE_MIN, self.speed_scale / SPEED_STEP)

        def held(*names: str) -> bool:
            return any(k in keys for k in names)

        forward = held("w", "up") - held("s", "down")
        left = held("a", "left") - held("d", "right")
        self.command = Twist(Vector3(x=forward * LINEAR_SPEED * self.speed_scale), Vector3(z=left * ANGULAR_SPEED * self.speed_scale))

    @property
    def command(self) -> Twist:
        return self.opencr.command

    @command.setter
    def command(self, twist: Twist) -> None:
        self.opencr.write(twist)  # 객체째 교체라 다른 스레드에서 넣어도 안전
