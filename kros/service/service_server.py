from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any, Callable

from .service_type import ServiceType


class ServiceServer:
    """서비스 서버 하나. 전용 워커 스레드 1개라 요청은 도착 순서대로 하나씩 처리된다."""

    def __init__(self, name: str, srv_type: ServiceType[Any, Any], handler: Callable[[Any], Any]) -> None:
        self.name: str = name
        self.srv_type: ServiceType[Any, Any] = srv_type
        self.handler: Callable[[Any], Any] = handler
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"srv:{name}")

    def submit(self, request: Any) -> Future[Any]:
        return self._executor.submit(self._handle, request)

    def _handle(self, request: Any) -> Any:
        response = self.handler(request)
        if not isinstance(response, self.srv_type.response):
            raise TypeError(
                f"service {self.name!r} must return {self.srv_type.response.__name__}, got {type(response).__name__}"
            )
        return response

    def close(self, wait: bool) -> None:
        # wait=False면 큐에 남은 요청은 취소된다 (호출자는 CancelledError)
        self._executor.shutdown(wait=wait, cancel_futures=not wait)
