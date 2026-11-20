from collections.abc import Mapping
from concurrent.futures import Future
from typing import Callable, TypeVar

from kros.action import ActionClient, ActionServer, ActionType, ServerGoalHandle
from kros.context import Context
from kros.log import LOG_TOPIC_NAME, NodeLogger
from kros.msgs.rcl_interfaces import LogLevel
from kros.parameter import OnChange, Parameter, ParameterValue, Validator
from kros.timer import Timer
from kros.topic import QOS_DEFAULT, QoS, Subscription
from kros.service import ServiceServer, ServiceType

T = TypeVar("T")
P = TypeVar("P", bound=ParameterValue)
Req = TypeVar("Req")
Res = TypeVar("Res")
Goal = TypeVar("Goal")
Feedback = TypeVar("Feedback")
Result = TypeVar("Result")


class Node:
    """kros 노드의 기본형: Context를 반드시 가진다. 이름은 기본이 클래스 이름이다 (같은 Context에서 겹치면 ValueError:
    같은 클래스를 여럿 만들 때는 name=을 따로 준다). 파라미터를 선언해서 가진다.

    parameters: 이 노드 인스턴스의 파라미터 시작 값 {이름: 값}. 선언한 기본값보다 우선이고, Kros(parameters=...)가 준 값보다는 약하다.
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        self.name: str = name or type(self).__name__
        self.context: Context = context
        context.register_node(self.name)
        if parameters:
            context.parameter_manager.set_node_overrides(self.name, parameters)
        # 모든 노드가 선언하는 파라미터 (ROS의 use_sim_time처럼): 로그 수준. 실행 중에 바꾸면 로거가 바로 따른다
        self._log_level = self.declare_parameter("log_level", "INFO", validator=lambda level: level.upper() in LogLevel.__members__)
        self._logger = NodeLogger(self.name, lambda log: self.publish(LOG_TOPIC_NAME, log), self.now, lambda: LogLevel[self._log_level.value.upper()])

    def declare_parameter(self, name: str, default: P, validator: Validator[P] | None = None, on_change: OnChange[P] | None = None) -> Parameter[P]:
        """파라미터를 선언한다. 타입은 default의 타입으로 고정된다. 돌려받은 Parameter의 value가 항상 현재 값이다.
        validator(새 값) -> bool로 값을 거절할 수 있고, on_change(새 값)은 값이 바뀐 뒤에 불린다."""
        return self.context.parameter_manager.declare(self.name, name, default, validator, on_change)

    def get_parameter(self, name: str) -> ParameterValue:
        return self.context.parameter_manager.get(self.name, name)

    def set_parameter(self, name: str, value: ParameterValue) -> None:
        self.context.parameter_manager.set(self.name, name, value)

    def get_logger(self) -> NodeLogger:
        """이 노드의 로거: 파이썬 logging과 rosout 토픽으로 나간다. 수준은 log_level 파라미터(기본 INFO)가 정한다."""
        return self._logger

    def now(self) -> float:
        """kros의 현재 시각(초): 실제 시간 또는 시뮬레이션 시간 (Kros의 time_source가 정한다). 메시지 시각 등에 쓴다."""
        return self.context.timer_manager.now()

    def create_timer(self, period: float, callback: Callable[[], None]) -> Timer:
        """period(초)마다 callback()을 부른다. 첫 호출은 period 뒤. 콜백은 이 타이머 전용 스레드에서 순서대로 돈다."""
        return self.context.timer_manager.create(period, callback)

    def destroy_timer(self, timer: Timer) -> None:
        self.context.timer_manager.destroy(timer)

    def publish(self, topic: str, msg: T, qos: QoS = QOS_DEFAULT) -> None:
        """토픽에 발행한다. qos=QOS_LATCHED처럼 TRANSIENT_LOCAL로 내면 마지막 메시지를 기억해 늦게 구독한 쪽에게도 전달한다."""
        self.context.topic_manager.publish(topic, msg, qos)

    def subscribe(self, topic: str, msg_type: type[T], callback: Callable[[T], None], qos: QoS = QOS_DEFAULT) -> Subscription:
        """토픽을 구독한다. qos.depth로 대기 큐 길이를 제한하고(넘치면 오래된 것을 버림), TRANSIENT_LOCAL이면 늦게 구독해도 기억된 메시지를 받는다."""
        return self.context.topic_manager.subscribe(topic, msg_type, callback, qos)

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
        return self.context.action_manager.create_server(name, action_type, execute, goal_callback)

    def destroy_action_server(self, server: ActionServer[Goal, Feedback, Result]) -> None:
        self.context.action_manager.destroy_server(server)

    def create_action_client(
        self, name: str, action_type: ActionType[Goal, Feedback, Result]
    ) -> ActionClient[Goal, Feedback, Result]:
        return self.context.action_manager.create_client(name, action_type)

    def destroy_action_client(self, client: ActionClient[Goal, Feedback, Result]) -> None:
        self.context.action_manager.destroy_client(client)

    def __repr__(self) -> str:
        return f"Node({self.name!r})"
