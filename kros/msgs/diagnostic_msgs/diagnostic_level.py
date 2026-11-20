from enum import IntEnum


class DiagnosticLevel(IntEnum):
    """진단 수준 (ROS diagnostic_msgs/DiagnosticStatus의 OK/WARN/ERROR/STALE). 숫자가 클수록 나쁘다."""

    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3  # 한동안 소식이 없어서 상태를 모른다
