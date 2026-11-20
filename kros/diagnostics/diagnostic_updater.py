import logging
import threading
from collections.abc import Callable

from kros.msgs.diagnostic_msgs import DiagnosticArray, DiagnosticLevel, DiagnosticStatus
from kros.msgs.std_msgs import Header
from kros.node import Node
from kros.timer import Timer

from .status_wrapper import DiagnosticStatusWrapper
from .topics import DIAGNOSTICS_TOPIC_NAME

logger = logging.getLogger(__name__)
Task = Callable[[DiagnosticStatusWrapper], None]


class DiagnosticUpdater:
    """노드가 자기 상태를 주기적으로 diagnostics 토픽에 내보내게 해 주는 도우미 (ROS의 diagnostic_updater).

    add(이름, task)로 진단 작업을 등록하면 period(초)마다 모든 작업을 돌려 한 DiagnosticArray로 발행한다. 작업은 상태 빌더에
    summary/add로 수준과 값을 적는다. 작업이 예외를 내면 그 진단만 ERROR로 내보내고 나머지는 그대로 나간다.
    진단 이름은 "노드이름: 작업이름"이다. 작업은 이 타이머의 워커 스레드에서 돈다 (읽는 값은 다른 스레드가 바꿀 수 있다).
    """

    def __init__(self, node: Node, period: float = 1.0, hardware_id: str = "") -> None:
        self._node = node
        self.hardware_id = hardware_id
        self._lock = threading.Lock()
        self._tasks: dict[str, Task] = {}
        self._timer: Timer | None = None
        self.period = period
        self._timer = node.create_timer(period, self._tick)

    def add(self, name: str, task: Task) -> None:
        with self._lock:
            if name in self._tasks:
                raise ValueError(f"진단 작업 이름이 겹친다: {name!r}")
            self._tasks[name] = task

    def remove(self, name: str) -> None:
        with self._lock:
            self._tasks.pop(name, None)

    def set_period(self, period: float) -> None:
        """발행 주기를 바꾼다 (타이머를 새로 만든다). 0 이하면 ValueError이고 기존 주기를 유지한다."""
        new = self._node.create_timer(period, self._tick)  # period가 잘못되면 여기서 예외: 기존 타이머는 그대로
        with self._lock:
            old, self._timer, self.period = self._timer, new, period
        if old is not None:
            old.cancel()

    def update(self) -> DiagnosticArray | None:
        """모든 진단 작업을 지금 돌려 발행한다 (타이머가 부르고, 직접 불러도 된다). 작업이 없으면 아무것도 안 낸다."""
        with self._lock:
            tasks = list(self._tasks.items())
        if not tasks:
            return None
        statuses = tuple(self._run(name, task) for name, task in tasks)
        array = DiagnosticArray(Header(self._node.now()), statuses)
        self._node.publish(DIAGNOSTICS_TOPIC_NAME, array)
        return array

    def _tick(self) -> None:
        self.update()

    def close(self) -> None:
        with self._lock:
            timer, self._timer = self._timer, None
        if timer is not None:
            timer.cancel()

    def _run(self, name: str, task: Task) -> DiagnosticStatus:
        full_name = f"{self._node.name}: {name}"
        wrapper = DiagnosticStatusWrapper()
        try:
            task(wrapper)
        except Exception as error:
            logger.exception("diagnostic task %r failed", full_name)
            wrapper = DiagnosticStatusWrapper()
            wrapper.summary(DiagnosticLevel.ERROR, f"진단 작업이 실패했다: {type(error).__name__}: {error}")
        return wrapper.build(full_name, self.hardware_id)
