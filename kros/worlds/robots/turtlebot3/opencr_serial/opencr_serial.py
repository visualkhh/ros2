import logging
import math

from kros.worlds.robots.turtlebot3.opencr_serial import protocol
from kros.worlds.robots.turtlebot3.opencr_serial.board_status import BoardStatus
from kros.worlds.robots.turtlebot3.opencr_serial.serial_link import SerialLink
from kros.worlds.robots.turtlebot3.opencr_serial.serial_mpu9250 import SerialMpu9250
from kros.worlds.robots.turtlebot3.opencr_serial.serial_wheel import SerialWheel
from kros.hardwares.controllers.diff_drive_board import DiffDriveBoard
from kros.hardwares.controllers.diff_drive import wheel_speeds
from kros.hardwares.hardware import HardwareError
from kros.msgs.geometry_msgs import Twist
from kros.worlds.clock import Clock

logger = logging.getLogger(__name__)
COMMAND_PERIOD = 0.1  # 초. 펌웨어는 500ms 동안 C가 없으면 바퀴를 세우므로 그 안에 계속 보낸다
MAX_WHEEL = 8.0  # rad/s, 펌웨어가 한 바퀴씩 자르는 한계. 넘으면 두 바퀴를 같은 비율로 줄인다 (호의 모양을 지킨다)


class OpenCrSerial(DiffDriveBoard):
    """진짜 OpenCR 보드를 USB 시리얼로 쓰는 구현 (펌웨어 firmware/opencr/tb3_wheels). OpenCr(시뮬레이션)와 같은 자리에 쓴다:
    부품은 left/right(SerialWheel)와 imu(SerialMpu9250)이고, 월드를 모른다 (시계 신호만 받는다).

    - 명령: write(Twist) -> 역기구학으로 바퀴 목표 속도(left/right.write) -> 주기마다 C 줄로 보낸다.
    - 읽기: 읽기 스레드가 S(바퀴 위치/속도 -> left/right.position/velocity, 보드 상태 -> status), I(-> imu), E(바퀴 에러 -> 바퀴의 fail 기록)를 파싱한다.
    - 팔(OpenMANIPULATOR): 아직 다루지 않는다. 펌웨어가 보내는 M 줄만 보고 팔이 연결됐는지(arm_connected: 모든 값이 nan이면 False, 아직 모르면 None)만 안다.
    - start()/stop()으로 읽기 스레드를 켜고 끈다 (stop은 바퀴를 세우는 C,0,0을 보낸다). 포트가 없어도 start()는 되고, 연결되면 알아서 이어진다.
    """

    model = "OpenCR1.0"
    imu: SerialMpu9250  # 인터페이스(Imu)보다 좁게: 읽기 스레드가 update()로 새 줄을 넣는다

    def __init__(
        self,
        wheel_radius: float,
        wheel_separation: float,
        port: str = "/dev/opencr",
        baud: int = 115200,
        period: float | None = COMMAND_PERIOD,
        left_dir: float = 1.0,
        right_dir: float = 1.0,
        max_wheel: float = MAX_WHEEL,
    ) -> None:
        super().__init__(period)
        self.wheel_radius, self.wheel_separation = wheel_radius, wheel_separation
        # 이 로봇은 두 모터 모두 +속도 = 뒤로 (robot.yaml left_dir/right_dir = -1). 명령과 엔코더에 같은 부호를 적용해
        # kros 안에서는 +가 앞으로가 되게 한다 (DiffDriveOdometry가 그대로 쓴다).
        self.left_dir, self.right_dir = left_dir, right_dir
        self.max_wheel = max_wheel
        self.left, self.right = SerialWheel(), SerialWheel()
        self.imu = SerialMpu9250()
        self.add(self.left, "left_wheel")
        self.add(self.right, "right_wheel")
        self.add(self.imu, "imu")
        self.status: BoardStatus | None = None  # 가장 최근 S 줄 (배터리, 온도, 전류, uptime)
        self.arm_connected: bool | None = None
        self.bad_lines = 0  # 못 읽은 줄 수 (펌웨어가 모르는 종류로 보낸 줄은 세지 않는다)
        self._command = Twist()
        self._link = SerialLink(port, baud, self._on_line)
        self._errors = (0, 0)
        self._send_failed = False

    @property
    def connected(self) -> bool:
        return self._link.connected

    def start(self) -> None:
        self._link.start()

    def stop(self) -> None:
        self._link.write_line(protocol.format_command(0.0, 0.0))  # 정지를 알리고 끈다
        self._link.stop()

    @property
    def command(self) -> Twist:
        return self._command

    def write(self, command: Twist) -> None:
        """몸체 속도 명령을 내린다 (바퀴 목표로 바꿔 두면 다음 주기에 보낸다)."""
        self._command = command
        left, right = wheel_speeds(command, self.wheel_radius, self.wheel_separation, self.max_wheel)
        self.left.write(left * self.left_dir)
        self.right.write(right * self.right_dir)

    def on_period(self, clock: Clock) -> None:
        sent = self._link.write_line(protocol.format_command(self.left.command, self.right.command))
        if not sent and not self._send_failed:  # 연결이 끊긴 사실은 한 번만 기록한다 (주기마다 세지 않는다)
            self.fail(HardwareError("명령을 보내지 못했다 (연결 안 됨)"))
        self._send_failed = not sent

    # ---- 읽기 스레드 ------------------------------------------------------------------------

    def _on_line(self, line: str) -> None:
        parsed = protocol.split_line(line)
        if parsed is None:
            self.bad_lines += 1
            return
        kind, values = parsed
        if kind == "S":
            status = protocol.parse_status(values)
            if status is None:
                self.bad_lines += 1
                return
            self.status = status
            self.left.position, self.left.velocity = status.left_position * self.left_dir, status.left_velocity * self.left_dir
            self.right.position, self.right.velocity = status.right_position * self.right_dir, status.right_velocity * self.right_dir
        elif kind == "I":
            imu = protocol.parse_imu(values)
            if imu is None:
                self.bad_lines += 1
            else:
                self.imu.update(imu)
        elif kind == "E":
            errors = protocol.parse_errors(values)
            if errors is None:
                self.bad_lines += 1
            elif errors != self._errors:  # 변할 때만 기록한다 (에러 중에는 펌웨어가 1Hz로 되풀이한다)
                self._errors = errors
                for wheel, code in zip((self.left, self.right), errors):
                    if code != 0:
                        wheel.fail(HardwareError(f"바퀴 에러: {protocol.error_text(code)}"))
        elif kind == "M":
            self.arm_connected = protocol.arm_connected(values)

    def __repr__(self) -> str:
        return f"<OpenCrSerial {self._link.port} connected={self.connected}>"
