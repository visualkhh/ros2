import logging
import threading
import time
from collections.abc import Callable

from kros.worlds.world import World

logger = logging.getLogger(__name__)


class SimulationRunner:
    """월드를 화면(렌더링)과 따로, 고정 간격(step_hz)으로 실시간에 맞춰 돌린다. 월드만 안다 (로봇이 뭔지는 모른다). kros와 무관하다.

    월드는 이 스레드만 만진다(시작 전의 add 제외). 월드를 읽어야 하는 쪽(화면에 줄 스냅샷 만들기, 시간 알리기 ...)은
    add_listener로 콜백을 달아 스텝이 끝날 때마다 이 스레드에서 읽는다. 바깥에서 넣는 값은 객체째 교체해서 넣는다(예: 로봇의 command).
    고정 간격이라 화면 프레임률과 상관없이 물리가 항상 같게 돈다. 월드를 다른 프로세스로 옮길 때는 이 클래스만 바꾸면 된다.
    """

    def __init__(self, world: World, step_hz: float = 60.0) -> None:
        self.world = world
        self.dt = 1.0 / step_hz
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._listeners: list[Callable[[World], None]] = []

    def add_listener(self, callback: Callable[[World], None]) -> None:
        """스텝이 끝날 때마다 callback(world)가 불린다 (시뮬레이션 스레드에서, 등록한 순서대로). 월드를 읽는 건 여기서만 안전하다."""
        self._listeners.append(callback)

    def step_once(self) -> None:
        """한 스텝 진행하고 리스너들을 부른다 (스레드 없이 돌리거나 테스트할 때)."""
        self.world.step(self.dt)
        for callback in self._listeners:
            callback(self.world)

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="simulation", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def _loop(self) -> None:
        next_time = time.perf_counter()
        try:
            while not self._stop.is_set():
                self.step_once()
                next_time += self.dt
                delay = next_time - time.perf_counter()
                if delay > 0:
                    self._stop.wait(delay)
                elif delay < -0.25:  # 너무 밀렸다(계산이 느리다): 따라잡지 않고 버린다
                    next_time = time.perf_counter()
        except Exception:
            logger.exception("시뮬레이션 스레드가 죽었다")
