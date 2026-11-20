from dataclasses import dataclass

from .log_level import LogLevel


@dataclass(frozen=True)
class Log:
    """rosout 토픽에 나가는 로그 한 줄 (ROS rcl_interfaces/Log 흉내): 시각(kros 시간), 수준, 노드 이름, 메시지, 부른 곳(파일, 함수, 줄)."""

    stamp: float
    level: LogLevel
    name: str
    msg: str
    file: str = ""
    function: str = ""
    line: int = 0
