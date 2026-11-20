from dataclasses import dataclass


@dataclass(frozen=True)
class BoardStatus:
    """OpenCR가 20Hz로 보내는 S 줄 한 개. 위치는 연속 회전각(rad), 속도 rad/s, 전류 A, 온도 ℃, 배터리 V. 펌웨어가 안 보낸 값은 nan이다."""

    left_position: float
    left_velocity: float
    right_position: float
    right_velocity: float
    uptime_ms: float
    left_current: float = float("nan")
    right_current: float = float("nan")
    left_temperature: float = float("nan")
    right_temperature: float = float("nan")
    battery_voltage: float = float("nan")
