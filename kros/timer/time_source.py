import threading
import time
from abc import ABC, abstractmethod
from collections.abc import Callable


class TimeSource(ABC):
    """kros의 시간 원천. now()는 메시지 시각 등에 쓰는 시각(초), monotonic()은 타이머 스케줄에 쓰는 단조 시각(초).
    실제 시간(SystemTimeSource)이거나 시뮬레이션이 밀어 주는 시간(SimulatedTimeSource)이다 (ROS의 use_sim_time과 같은 역할)."""

    realtime: bool  # True면 시간이 스스로 흐른다 (타이머는 기다렸다가 깨운다). False면 바깥이 set할 때만 흐른다

    @abstractmethod
    def now(self) -> float: ...

    @abstractmethod
    def monotonic(self) -> float: ...

    def add_listener(self, callback: Callable[[], None]) -> None:
        """시간이 바뀔 때마다 불리는 콜백 등록 (바깥이 시간을 밀어 주는 소스만 의미가 있다)."""


class SystemTimeSource(TimeSource):
    realtime = True

    def now(self) -> float:
        return time.time()

    def monotonic(self) -> float:
        return time.monotonic()


class SimulatedTimeSource(TimeSource):
    """시뮬레이션이 set()/advance()로 밀어 주는 시간. 시간이 되돌아가면 ValueError. 시간이 바뀌면 listener를 부른다 (타이머 판정)."""

    realtime = False

    def __init__(self, start: float = 0.0) -> None:
        self._time = start
        self._lock = threading.Lock()
        self._listeners: list[Callable[[], None]] = []

    def now(self) -> float:
        with self._lock:
            return self._time

    def monotonic(self) -> float:
        return self.now()

    def add_listener(self, callback: Callable[[], None]) -> None:
        with self._lock:
            self._listeners.append(callback)

    def set(self, t: float) -> None:
        with self._lock:
            if t < self._time:
                raise ValueError(f"시간이 되돌아갈 수 없다: {self._time} -> {t}")
            self._time = t
            listeners = list(self._listeners)
        for callback in listeners:  # set을 부른 스레드에서 바로 타이머를 판정한다 (스케줄 스레드를 기다리지 않는다)
            callback()

    def advance(self, dt: float) -> None:
        self.set(self.now() + dt)
