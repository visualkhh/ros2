import threading
from concurrent.futures import Future
from typing import Any, Callable, TypeVar

from .service_server import ServiceServer
from .service_type import ServiceType

Req = TypeVar("Req")
Res = TypeVar("Res")


class ServiceExecutor:
    """서비스 이름 -> 서버 레지스트리 (한 프로세스 안).

    - 서비스 이름당 서버는 하나. 중복 등록은 ValueError.
    - 호출/등록 모두 ServiceType을 선언해야 하고, 다르면 TypeError.
    - 서버가 없으면 LookupError. 핸들러 예외는 호출자의 result()로 전달된다.
    - 서버 스레드 안에서 같은 서비스를 블로킹 호출하면 데드락 (timeout으로 TimeoutError).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._servers: dict[str, ServiceServer] = {}

    def create_service(
        self, name: str, srv_type: ServiceType[Req, Res], handler: Callable[[Req], Res]
    ) -> ServiceServer:
        with self._lock:
            if name in self._servers:
                raise ValueError(f"service {name!r} already has a server")
            server = ServiceServer(name, srv_type, handler)
            self._servers[name] = server
        return server

    def destroy_service(self, server: ServiceServer) -> None:
        with self._lock:
            if self._servers.get(server.name) is server:
                del self._servers[server.name]
        server.close(wait=False)

    def call_async(self, name: str, srv_type: ServiceType[Req, Res], request: Req) -> Future[Res]:
        with self._lock:
            server = self._servers.get(name)
        if server is None:
            raise LookupError(f"no server for service {name!r}")
        if server.srv_type != srv_type:
            raise TypeError(f"service {name!r} is {server.srv_type.name}, got {srv_type.name}")
        if not isinstance(request, srv_type.request):
            raise TypeError(f"service {name!r} takes {srv_type.request.__name__}, got {type(request).__name__}")
        try:
            return server.submit(request)
        except RuntimeError:  # 호출 도중 destroy/shutdown된 서버
            raise LookupError(f"service {name!r} is gone") from None

    def call(
        self, name: str, srv_type: ServiceType[Req, Res], request: Req, timeout: float | None = None
    ) -> Res:
        return self.call_async(name, srv_type, request).result(timeout)

    def shutdown(self) -> None:
        with self._lock:
            servers = list(self._servers.values())
            self._servers.clear()
        for server in servers:
            server.close(wait=True)
ServiceManager = ServiceExecutor
