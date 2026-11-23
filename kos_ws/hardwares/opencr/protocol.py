"""OpenCR 펌웨어(firmware/opencr/tb3_wheels)의 시리얼 프로토콜: 줄 단위 ASCII, 쉼표로 나눈 값, SI 단위. 파싱은 순수 함수다 (시리얼을 모른다).

- 보드가 보낸다: S(바퀴 상태), I(IMU), E(바퀴 하드웨어 에러), M(팔 상태), F(팔 에러). 모르는 줄은 무시한다 (펌웨어가 줄을 추가해도 안 깨진다).
- 우리가 보낸다: C,<좌 rad/s>,<우 rad/s> (500ms 안에 안 보내면 펌웨어가 바퀴를 세운다).
"""
import math

from .board_status import BoardStatus
from kros.msgs.geometry_msgs import Quaternion, Vector3
from kros.msgs.sensor_msgs import Imu
from kros.msgs.std_msgs import Header

ERROR_BITS = {1: "입력 전압", 4: "과열", 8: "모터 엔코더", 16: "전기 충격", 32: "과부하"}
ARM_VALUES = 18  # M 줄: 관절 6개 x (위치, 속도, 전류). 뒤에 온도 6개


def error_text(code: int) -> str:
    """E 줄의 에러 비트마스크를 글로. 0은 정상, 음수는 무응답."""
    if code == 0:
        return "정상"
    if code < 0:
        return "무응답"
    return "+".join(name for bit, name in ERROR_BITS.items() if code & bit) or str(code)


def split_line(line: str) -> tuple[str, list[float]] | None:
    """한 줄을 (종류, 값들)로. 숫자가 아닌 값이 있으면 None (nan, inf는 숫자로 읽는다)."""
    kind, *values = line.strip().split(",")
    try:
        return kind, [float(v) for v in values]
    except ValueError:
        return None


def parse_status(values: list[float]) -> BoardStatus | None:
    """S 줄의 값들 -> BoardStatus. 최소 5개(위치/속도 둘씩 + uptime)가 있어야 한다."""
    if len(values) < 5:
        return None
    return BoardStatus(*values[:10])


def parse_imu(values: list[float]) -> Imu | None:
    """I 줄(gx gy gz ax ay az qw qx qy qz) -> Imu 메시지. 시각은 받는 쪽이 찍는다 (여기서는 0)."""
    if len(values) < 10:
        return None
    gx, gy, gz, ax, ay, az, qw, qx, qy, qz = values[:10]
    return Imu(Header(0.0, "imu_link"), Quaternion(qx, qy, qz, qw), Vector3(gx, gy, gz), Vector3(ax, ay, az))


def parse_errors(values: list[float]) -> tuple[int, int] | None:
    """E 줄 -> (왼쪽 에러, 오른쪽 에러). 0 정상, 음수 무응답, 나머지는 비트마스크."""
    if len(values) < 2 or not all(math.isfinite(v) for v in values[:2]):
        return None
    return int(values[0]), int(values[1])


def arm_connected(values: list[float]) -> bool | None:
    """M 줄 -> 팔이 연결돼 있는가. 팔이 없으면 펌웨어가 읽은 값을 전부 nan으로 보낸다. 줄이 짧으면 None(모르겠다)."""
    if len(values) < ARM_VALUES:
        return None
    return any(not math.isnan(v) for v in values[:ARM_VALUES])


def format_command(left: float, right: float) -> str:
    """바퀴 목표 속도(rad/s) -> C 줄."""
    return f"C,{left:.4f},{right:.4f}"
