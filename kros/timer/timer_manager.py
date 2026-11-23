import logging
import threading
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor

from .time_source import SystemTimeSource, TimeSource
from .timer import Timer

logger = logging.getLogger(__name__)
EPSILON = 1e-9  # 주기 판정의 부동소수점 오차 허용 (간격을 빼서 비교하면 한 번씩 밀린다)


class _Entry:
    def __init__(self, timer: Timer, next_due: float) -> None:
        self.timer = timer
        self.next_due = next_due
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="timer")
        self.busy = False  # 지난 콜백이 아직 실행(또는 대기) 중
        self.canceled = False


class TimerManager:
    """타이머 레지스트리와 스케줄 (한 프로세스 안). TopicExecutor/ServiceExecutor와 같은 결이다.

    - 시간은 TimeSource가 준다: 실제 시간이면 스케줄 스레드 하나가 기다렸다 깨우고, 시뮬레이션 시간이면 시간을 set하는 쪽이
      그 스레드에서 바로 타이머를 판정한다 (스레드 없이 결정적으로 동작).
    - 첫 호출은 만든 뒤 period가 지났을 때. 시간이 한꺼번에 건너뛰면 한 번만 부르고 나머지는 missed로 센다 (박자는 유지).
    - 콜백은 타이머마다 전용 워커 스레드 하나에서 돈다. 지난 콜백이 아직 안 끝났으면 새 호출은 버리고 missed로 센다.
    """

    def __init__(self, time_source: TimeSource | None = None) -> None:
        self.time_source: TimeSource = time_source or SystemTimeSource()
        self._cond = threading.Condition()
        self._entries: list[_Entry] = []
        self._inflight = 0  # 실행 중이거나 대기 중인 콜백 수 (join용)
        self._stopped = False
        self._thread: threading.Thread | None = None
        self.time_source.add_listener(self._fire_due)

    def now(self) -> float:
        return self.time_source.now()

    def create(self, period: float, callback: Callable[[], None]) -> Timer:
        if period <= 0:
            raise ValueError(f"타이머 주기는 0보다 커야 한다: {period}")
        timer = Timer(period, callback, self.destroy)
        with self._cond:
            if self._stopped:
                raise RuntimeError("타이머 매니저가 종료됐다")
            self._entries.append(_Entry(timer, self.time_source.monotonic() + period))
            if self.time_source.realtime and self._thread is None:
                self._thread = threading.Thread(target=self._loop, name="timer-scheduler", daemon=True)
                self._thread.start()
            self._cond.notify_all()
        return timer

    def destroy(self, timer: Timer) -> None:
        with self._cond:
            timer.is_canceled = True
            entry = next((e for e in self._entries if e.timer is timer), None)
            if entry is None:
                return
            entry.canceled = True
            self._entries.remove(entry)
            self._cond.notify_all()
        entry.executor.shutdown(wait=False)  # 이미 대기 중인 콜백은 canceled를 보고 건너뛴다

    def timer_count(self) -> int:
        with self._cond:
            return len(self._entries)

    def join(self, timeout: float | None = None) -> bool:
        """지금 실행 중이거나 대기 중인 콜백이 모두 끝날 때까지 기다린다. 끝났으면 True."""
        with self._cond:
            return self._cond.wait_for(lambda: self._inflight == 0, timeout)

    def shutdown(self) -> None:
        with self._cond:
            self._stopped = True
            entries, self._entries = self._entries, []
            thread, self._thread = self._thread, None
            for entry in entries:
                entry.canceled = True
                entry.timer.is_canceled = True
            self._cond.notify_all()
        if thread is not None and thread is not threading.current_thread():
            thread.join()
        for entry in entries:
            entry.executor.shutdown(wait=True)

    def _fire_due(self) -> None:
        now = self.time_source.monotonic()
        run: list[_Entry] = []
        with self._cond:
            if self._stopped:
                return
            for entry in self._entries:
                if entry.next_due > now + EPSILON:
                    continue
                steps = 0
                while entry.next_due <= now + EPSILON:  # 박자를 유지한 채 다음 주기로
                    entry.next_due += entry.timer.period
                    steps += 1
                if entry.busy:
                    entry.timer.missed += steps  # 지난 콜백이 아직 안 끝났다: 이번 호출은 버린다
                    continue
                entry.timer.missed += steps - 1
                entry.busy = True
                self._inflight += 1
                run.append(entry)
        for entry in run:
            try:
                entry.executor.submit(self._run, entry)
            except RuntimeError:  # 그 사이 destroy된 타이머
                self._finished(entry)

    def _run(self, entry: _Entry) -> None:
        try:
            if not entry.canceled:
                entry.timer.callback()
                entry.timer.calls += 1
        except Exception:
            logger.exception("timer callback failed (period=%s)", entry.timer.period)
        finally:
            self._finished(entry)

    def _finished(self, entry: _Entry) -> None:
        with self._cond:
            entry.busy = False
            self._inflight -= 1
            self._cond.notify_all()

    def _loop(self) -> None:
        """실제 시간용 스케줄 스레드: 가장 가까운 주기까지 기다렸다가 깨운다."""
        while True:
            with self._cond:
                if self._stopped:
                    return
                due = min((e.next_due for e in self._entries), default=None)
                remaining = None if due is None else due - self.time_source.monotonic()
                if remaining is None or remaining > 0:
                    self._cond.wait(remaining)  # 새 타이머/삭제/종료 때도 깨어난다
                    continue
            self._fire_due()

