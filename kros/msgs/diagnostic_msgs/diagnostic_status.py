from dataclasses import dataclass

from .diagnostic_level import DiagnosticLevel
from .key_value import KeyValue


@dataclass(frozen=True)
class DiagnosticStatus:
    """진단 하나 (ROS diagnostic_msgs/DiagnosticStatus 흉내): 수준, 이름, 한 줄 설명, 장치 식별자, 자세한 값들."""

    level: DiagnosticLevel
    name: str
    message: str = ""
    hardware_id: str = ""
    values: tuple[KeyValue, ...] = ()
