import logging
import threading
from collections import deque
from typing import Any, Callable, TypeVar

from .qos import QOS_DEFAULT, Durability, QoS
from .subscription import Subscription

T = TypeVar("T")
log = logging.getLogger(__name__)


class TopicExecutor:
    """토픽별 구독자 목록을 들고 메시지를 전달하는 중앙 매니저 (한 프로세스 안).

    - 토픽은 처음 쓰일 때 메시지 타입이 고정된다. 다른 타입이면 TypeError.
    - publish는 즉시 반환하고, 구독자마다 자기 스레드에서 동시에 콜백을 실행한다.
      한 콜백이 예외를 던져도 로그만 남기고 다른 구독자에는 영향이 없다.
    - QoS: 구독의 depth는 대기 큐 길이 제한(넘치면 가장 오래된 것을 버림), TRANSIENT_LOCAL로 발행한 메시지는 기억해 뒀다가
      TRANSIENT_LOCAL로 나중에 구독한 쪽에게도 먼저 전달한다 (latch). 기본은 무제한/휘발성이라 아무것도 버리지도 기억하지도 않는다.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._subs: dict[str, list[Subscription]] = {}
        self._types: dict[str, type] = {}
        self._history: dict[str, deque[Any]] = {}  # TRANSIENT_LOCAL로 발행된 마지막 메시지들 (늦게 구독한 쪽을 위해)
        self._pending = 0  # 아직 실행 안 끝난 콜백 수
        self._idle = threading.Condition()

    def _bind(self, topic: str, msg_type: type) -> None:  # _lock을 잡고 호출
        bound = self._types.setdefault(topic, msg_type)
        if bound is not msg_type:
            raise TypeError(f"topic {topic!r} is {bound.__name__}, got {msg_type.__name__}")

    def subscribe(self, topic: str, msg_type: type[T], callback: Callable[[T], None], qos: QoS = QOS_DEFAULT) -> Subscription:
        sub = Subscription(topic, callback, qos, self._done)
        with self._lock:
            self._bind(topic, msg_type)
            if qos.durability is Durability.TRANSIENT_LOCAL:
                for msg in self._history.get(topic, ()):  # 기억해 둔 메시지를 새 메시지보다 먼저 (같은 락 안이라 끼어드는 publish가 없다)
                    self._dispatch(sub, msg)
            self._subs.setdefault(topic, []).append(sub)
        return sub

    def unsubscribe(self, sub: Subscription) -> None:
        with self._lock:
            subs = self._subs.get(sub.topic, [])
            if sub in subs:
                subs.remove(sub)
        sub.close(wait=False)

    def publish(self, topic: str, msg: T, qos: QoS = QOS_DEFAULT) -> None:
        with self._lock:
            self._bind(topic, type(msg))
            if qos.durability is Durability.TRANSIENT_LOCAL:
                assert qos.depth is not None  # QoS가 보장한다
                history = self._history.get(topic)
                if history is None or history.maxlen != qos.depth:  # 깊이가 바뀌면 최근 것을 남기고 다시 만든다
                    history = deque(history or (), maxlen=qos.depth)
                    self._history[topic] = history
                history.append(msg)
            subs = list(self._subs.get(topic, []))  # 복사본: 순회 중 subscribe/unsubscribe 안전
        for sub in subs:
            self._dispatch(sub, msg)

    def _dispatch(self, sub: Subscription, msg: Any) -> None:
        with self._idle:
            self._pending += 1
        if not sub.submit(msg):  # publish 도중 unsubscribe/shutdown된 구독
            self._done()

    def _done(self) -> None:
        with self._idle:
            self._pending -= 1
            if self._pending == 0:
                self._idle.notify_all()

    def join(self) -> None:
        """지금까지 publish된 모든 콜백(콜백 안에서 publish한 것 포함)이 끝날 때까지 기다린다."""
        with self._idle:
            self._idle.wait_for(lambda: self._pending == 0)

    def shutdown(self) -> None:
        """남은 콜백을 모두 끝내고 스레드를 정리한다. 이후 publish는 조용히 버려진다."""
        self.join()
        with self._lock:
            subs = [s for lst in self._subs.values() for s in lst]
            self._subs.clear()
            self._history.clear()
        for sub in subs:
            sub.close(wait=True)
TopicManager = TopicExecutor
