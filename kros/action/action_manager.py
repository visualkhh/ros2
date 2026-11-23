import threading
from typing import Any, Callable, TypeVar

from kros.service import ServiceManager
from kros.topic import TopicManager

from .action_client import ActionClient
from .action_server import ActionServer
from .action_type import ActionType
from .server_goal_handle import ServerGoalHandle

Goal = TypeVar("Goal")
Feedback = TypeVar("Feedback")
Result = TypeVar("Result")


class ActionExecutor:
    """액션 이름 -> 서버 레지스트리와 서버/클라이언트의 수명 관리 (한 프로세스 안). TopicManager/ServiceManager와 같은 결이다.

    액션은 토픽과 서비스 위에 올라가는 기능이라 두 매니저를 받아 쓴다 (둘의 연결은 Context가 한다).
    - 액션 이름당 서버는 하나. 중복 등록은 ValueError.
    - destroy_server: 실행 중인 goal은 취소 요청, 시작도 못 한 goal은 CANCELED로 끝내 클라이언트가 영원히 기다리지 않게 한다.
    - destroy_client: 결과를 기다리던 핸들은 ABORTED로 끝낸다.
    - shutdown: 클라이언트를 먼저(기다리던 결과는 ABORTED), 서버를 나중에 정리한다.
    """

    def __init__(self, topic_manager: TopicManager, service_manager: ServiceManager) -> None:
        self._topics = topic_manager
        self._services = service_manager
        self._lock = threading.Lock()
        self._servers: dict[str, ActionServer[Any, Any, Any]] = {}
        self._clients: list[ActionClient[Any, Any, Any]] = []

    def create_server(
        self,
        name: str,
        action_type: ActionType[Goal, Feedback, Result],
        execute: Callable[[ServerGoalHandle[Goal, Feedback]], Result],
        goal_callback: Callable[[Goal], bool] | None = None,
    ) -> ActionServer[Goal, Feedback, Result]:
        with self._lock:
            if name in self._servers:
                raise ValueError(f"action {name!r} already has a server")
            server = ActionServer(name, action_type, execute, self._topics, self._services, goal_callback)
            self._servers[name] = server
        return server

    def destroy_server(self, server: ActionServer[Goal, Feedback, Result]) -> None:
        with self._lock:
            if self._servers.get(server.name) is server:
                del self._servers[server.name]
        server.destroy()

    def create_client(self, name: str, action_type: ActionType[Goal, Feedback, Result]) -> ActionClient[Goal, Feedback, Result]:
        client = ActionClient(name, action_type, self._topics, self._services)
        with self._lock:
            self._clients.append(client)
        return client

    def destroy_client(self, client: ActionClient[Goal, Feedback, Result]) -> None:
        with self._lock:
            if client in self._clients:
                self._clients.remove(client)
        client.destroy()

    def server_names(self) -> list[str]:
        """지금 살아 있는 액션 서버의 이름들."""
        with self._lock:
            return sorted(self._servers)

    def shutdown(self) -> None:
        with self._lock:
            clients, servers = list(self._clients), list(self._servers.values())
            self._clients.clear()
            self._servers.clear()
        for client in clients:
            client.destroy()
        for server in servers:
            server.destroy()
ActionManager = ActionExecutor
