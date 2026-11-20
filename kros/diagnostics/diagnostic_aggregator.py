import dataclasses
import threading

from kros.context import Context
from kros.msgs.diagnostic_msgs import DiagnosticArray, DiagnosticLevel, DiagnosticStatus
from kros.node import Node

from .topics import DIAGNOSTICS_TOPIC_NAME


class DiagnosticAggregator(Node):
    """diagnostics 토픽을 구독해 이름별 최신 진단을 모으고, 한동안(stale_timeout) 소식이 없는 것은 STALE로 본다 (ROS의 diagnostic_aggregator가
    하는 일 중 가장 기본). 받은 시각은 이 노드가 받은 kros 시간이다. stale_timeout은 실행 중에도 바꿀 수 있는 파라미터다.
    """

    def __init__(self, context: Context, name: str | None = None, parameters: dict[str, bool | int | float | str] | None = None) -> None:
        super().__init__(context, name, parameters)
        self.stale_timeout = self.declare_parameter("stale_timeout", 5.0, validator=lambda seconds: seconds > 0)
        self._lock = threading.Lock()
        self._latest: dict[str, tuple[DiagnosticStatus, float]] = {}  # 이름 -> (최신 진단, 받은 시각)
        self.subscribe(DIAGNOSTICS_TOPIC_NAME, DiagnosticArray, self._on_array)

    def _on_array(self, array: DiagnosticArray) -> None:
        received = self.now()
        with self._lock:
            for status in array.status:
                self._latest[status.name] = (status, received)

    def statuses(self) -> dict[str, DiagnosticStatus]:
        """이름별 최신 진단. stale_timeout 넘게 소식이 없는 것은 수준이 STALE로 바뀐다."""
        now, timeout = self.now(), self.stale_timeout.value
        with self._lock:
            latest = dict(self._latest)
        result: dict[str, DiagnosticStatus] = {}
        for name, (status, received) in latest.items():
            age = now - received
            if age > timeout:
                status = dataclasses.replace(status, level=DiagnosticLevel.STALE, message=f"{age:.1f}초 동안 소식이 없다 (마지막: {status.message})")
            result[name] = status
        return result

    def worst_level(self) -> DiagnosticLevel:
        """가장 나쁜 수준. 아직 아무 진단도 못 받았으면 모르는 것이므로 STALE이다 (OK가 아니다)."""
        levels = [status.level for status in self.statuses().values()]
        return max(levels) if levels else DiagnosticLevel.STALE
