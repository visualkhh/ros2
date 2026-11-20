import logging
import threading
import time

from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.world import World

from .snapshot import Snapshot

logger = logging.getLogger(__name__)


class SimulationRunner:
    """월드를 화면(렌더링)과 따로, 고정 간격(step_hz)으로 실시간에 맞춰 돌린다. 화면은 latest(Snapshot)만 읽는다.

    월드는 이 스레드만 만진다(시작 전의 add 제외). 바깥에서 넣는 값은 robot.command뿐이고 객체째 교체라 안전하다.
    고정 간격이라 화면 프레임률과 상관없이 물리가 항상 같게 돈다. 월드를 다른 프로세스로 옮길 때는 이 클래스만 바꾸면 된다.
    """

    def __init__(self, world: World, robot: TurtleBot3, step_hz: float = 60.0) -> None:
        self.world, self.robot = world, robot
        self.dt = 1.0 / step_hz
        self.latest: Snapshot = self._snapshot()  # 한 줄 대입이라 읽는 쪽이 중간 값을 볼 일이 없다
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def step_once(self) -> Snapshot:
        """한 스텝 진행하고 스냅샷을 갱신한다 (스레드 없이 돌리거나 테스트할 때)."""
        self.world.step(self.dt)
        self.latest = self._snapshot()
        return self.latest

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

    def _snapshot(self) -> Snapshot:
        objects = self.world.objects
        return Snapshot(self.world.time, tuple(o.pose for o in objects), objects.index(self.robot), self.robot.command, (self.robot.drive.left.position, self.robot.drive.right.position), self.robot.lidar.latest, self.robot.lidar.scan_pose)
