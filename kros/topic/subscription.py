import logging
import threading
from collections import deque
from typing import Any, Callable

from .qos import QOS_DEFAULT, QoS

log = logging.getLogger(__name__)


class Subscription:
    """구독 하나. 전용 워커 스레드 1개 + 큐를 가져 같은 구독 안에서는 메시지 순서가 보장된다.

    큐 길이는 QoS.depth로 제한할 수 있다: 넘치면 가장 오래된 (아직 실행 전인) 메시지를 버리고 dropped로 센다.
    메시지 하나가 처리되거나 버려질 때마다 on_done()이 한 번 불린다 (TopicManager.join()의 계산용).
    """

    def __init__(self, topic: str, callback: Callable[[Any], None], qos: QoS = QOS_DEFAULT, on_done: Callable[[], None] | None = None) -> None:
        self.topic: str = topic
        self.callback: Callable[[Any], None] = callback
        self.qos: QoS = qos
        self.dropped = 0  # 큐가 넘쳐서 버려진 메시지 수 (사실만 노출한다)
        self._on_done = on_done or (lambda: None)
        self._cond = threading.Condition()
        self._queue: deque[Any] = deque()
        self._closed = False
        self._thread: threading.Thread | None = None

    def submit(self, msg: Any) -> bool:
        """메시지를 큐에 넣는다. 이미 닫힌 구독이면 False."""
        overflowed = False
        with self._cond:
            if self._closed:
                return False
            if self.qos.depth is not None and len(self._queue) >= self.qos.depth:
                self._queue.popleft()  # 가장 오래된 것을 버린다
                self.dropped += 1
                overflowed = True
            self._queue.append(msg)
            if self._thread is None:  # 첫 메시지가 올 때 스레드를 만든다
                self._thread = threading.Thread(target=self._loop, name=f"sub:{self.topic}", daemon=True)
                self._thread.start()
            self._cond.notify()
        if overflowed:
            self._on_done()  # 버려진 메시지도 "끝난" 것으로 센다
        return True

    def close(self, wait: bool) -> None:
        """wait=True: 큐에 남은 메시지를 모두 처리하고 스레드가 끝날 때까지 기다린다. wait=False: 큐에 남은 메시지는 버린다 (unsubscribe)."""
        with self._cond:
            self._closed = True
            discarded = 0
            if not wait:
                discarded = len(self._queue)
                self._queue.clear()
            thread = self._thread
            self._cond.notify_all()
        for _ in range(discarded):
            self._on_done()
        if wait and thread is not None and thread is not threading.current_thread():
            thread.join()

    def _loop(self) -> None:
        while True:
            with self._cond:
                while not self._queue and not self._closed:
                    self._cond.wait()
                if not self._queue:
                    return  # 닫혔고 큐도 비었다
                msg = self._queue.popleft()
            try:
                self.callback(msg)
            except Exception:
                log.exception("subscriber of %r failed", self.topic)
            finally:
                self._on_done()
