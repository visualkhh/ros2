from concurrent.futures import Future
from typing import Callable, TypeVar

from kros.action import ActionClient, ActionServer, ActionType, ServerGoalHandle
from kros.context import Context
from kros.topic import Subscription
from kros.service import ServiceServer, ServiceType

T = TypeVar("T")
Req = TypeVar("Req")
Res = TypeVar("Res")
Goal = TypeVar("Goal")
Feedback = TypeVar("Feedback")
Result = TypeVar("Result")


class Node:
    """kros 노드의 기본형: 이름과 Context를 반드시 가진다."""

    def __init__(self, name: str, context: Context) -> None:
        self.name: str = name
        self.context: Context = context

    def publish(self, topic: str, msg: T) -> None:
        self.context.topic_manager.publish(topic, msg)

    def subscribe(self, topic: str, msg_type: type[T], callback: Callable[[T], None]) -> Subscription:
        return self.context.topic_manager.subscribe(topic, msg_type, callback)

    def unsubscribe(self, subscription: Subscription) -> None:
        self.context.topic_manager.unsubscribe(subscription)

    def create_service(
        self, name: str, srv_type: ServiceType[Req, Res], handler: Callable[[Req], Res]
    ) -> ServiceServer:
        return self.context.service_manager.create_service(name, srv_type, handler)

    def destroy_service(self, server: ServiceServer) -> None:
        self.context.service_manager.destroy_service(server)

    def call_service(
        self, name: str, srv_type: ServiceType[Req, Res], request: Req, timeout: float | None = None
    ) -> Res:
        return self.context.service_manager.call(name, srv_type, request, timeout)

    def call_service_async(self, name: str, srv_type: ServiceType[Req, Res], request: Req) -> Future[Res]:
        return self.context.service_manager.call_async(name, srv_type, request)

    def create_action_server(
        self,
        name: str,
        action_type: ActionType[Goal, Feedback, Result],
        execute: Callable[[ServerGoalHandle[Goal, Feedback]], Result],
        goal_callback: Callable[[Goal], bool] | None = None,
    ) -> ActionServer[Goal, Feedback, Result]:
        return ActionServer(name, action_type, execute, self.context.topic_manager, self.context.service_manager, goal_callback)

    def create_action_client(
        self, name: str, action_type: ActionType[Goal, Feedback, Result]
    ) -> ActionClient[Goal, Feedback, Result]:
        return ActionClient(name, action_type, self.context.topic_manager, self.context.service_manager)

    def __repr__(self) -> str:
        return f"Node({self.name!r})"
