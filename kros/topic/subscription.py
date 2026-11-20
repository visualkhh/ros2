from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any, Callable


class Subscription:
    """구독 하나. 전용 워커 스레드 1개 + 큐를 가져 같은 구독 안에서는 메시지 순서가 보장된다."""

    def __init__(self, topic: str, callback: Callable[[Any], None]) -> None:
        self.topic: str = topic
        self.callback: Callable[[Any], None] = callback
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"sub:{topic}")

    def submit(self, fn: Callable[[], None]) -> Future[None]:
        return self._executor.submit(fn)

    def close(self, wait: bool) -> None:
        # wait=False면 큐에 남은 메시지는 버린다 (unsubscribe)
        self._executor.shutdown(wait=wait, cancel_futures=not wait)
