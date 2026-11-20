from enum import IntEnum


class LogLevel(IntEnum):
    """로그 수준 (ROS의 rcl_interfaces/Log 수준). 숫자는 파이썬 logging의 DEBUG/INFO/WARNING/ERROR/CRITICAL과 같다."""

    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40
    FATAL = 50
