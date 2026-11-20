from dataclasses import dataclass

from kros.msgs.std_msgs import Header

from .diagnostic_status import DiagnosticStatus


@dataclass(frozen=True)
class DiagnosticArray:
    """diagnostics 토픽에 나가는 메시지: 한 번에 여러 진단 (ROS diagnostic_msgs/DiagnosticArray 흉내)."""

    header: Header
    status: tuple[DiagnosticStatus, ...]
