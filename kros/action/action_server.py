import logging
import threading
from typing import Any, Callable, Generic, TypeVar

from kros.topic import TopicManager
from kros.service import ServiceManager

from .action_type import ActionType
from .cancel_goal_request import CancelGoalRequest
from .cancel_goal_response import CancelGoalResponse
from .action_event import ActionEvent
from .goal_message import GoalMessage
from .goal_status import GoalStatus
from .result_message import ResultMessage
from .send_goal_request import SendGoalRequest
from .send_goal_response import SendGoalResponse
from .server_goal_handle import ServerGoalHandle
from .wire import CANCEL_GOAL, SEND_GOAL, cancel_goal_service, event_topic, goal_topic, send_goal_service

Goal = TypeVar("Goal")
Feedback = TypeVar("Feedback")
Result = TypeVar("Result")
log = logging.getLogger(__name__)


class ActionServer(Generic[Goal, Feedback, Result]):
    """액션 서버. goal은 받은 순서대로 하나씩 실행된다 (실행은 전용 구독 스레드).

    execute(handle) -> Result 는 오래 걸려도 된다. handle.publish_feedback()으로 진행을 알리고
    handle.is_cancel_requested를 주기적으로 확인해 취소에 응한다.
    정상 반환 -> SUCCEEDED (취소 요청이 있었으면 CANCELED), 예외/잘못된 결과 타입 -> ABORTED.
    """

    def __init__(
        self,
        name: str,
        action_type: ActionType[Goal, Feedback, Result],
        execute: Callable[[ServerGoalHandle[Goal, Feedback]], Result],
        topic_manager: TopicManager,
        service_manager: ServiceManager,
        goal_callback: Callable[[Goal], bool] | None = None,
    ) -> None:
        self.name = name
        self._type = action_type
        self._execute = execute
        self._goal_callback = goal_callback
        self._topics = topic_manager
        self._sm = service_manager
        self._lock = threading.Lock()
        self._goals: dict[str, ServerGoalHandle[Goal, Feedback]] = {}
        self._send = service_manager.create_service(send_goal_service(name), SEND_GOAL, self._on_send_goal)
        self._cancel = service_manager.create_service(cancel_goal_service(name), CANCEL_GOAL, self._on_cancel_goal)
        self._goal_sub = topic_manager.subscribe(goal_topic(name), GoalMessage, self._run_goal)

    def _on_send_goal(self, request: SendGoalRequest) -> SendGoalResponse:
        if not isinstance(request.goal, self._type.goal):
            raise TypeError(f"action {self.name!r} goal is {self._type.goal.__name__}, got {type(request.goal).__name__}")
        if self._goal_callback is not None and not self._goal_callback(request.goal):
            return SendGoalResponse(accepted=False)
        handle: ServerGoalHandle[Goal, Feedback] = ServerGoalHandle(
            request.goal_id, request.goal, self._type, self._publish_feedback
        )
        with self._lock:
            self._goals[request.goal_id] = handle
        self._topics.publish(goal_topic(self.name), GoalMessage(request.goal_id, request.goal))
        return SendGoalResponse(accepted=True)

    def _on_cancel_goal(self, request: CancelGoalRequest) -> CancelGoalResponse:
        with self._lock:
            handle = self._goals.get(request.goal_id)
        if handle is None:  # 모르는 goal이거나 이미 끝남
            return CancelGoalResponse(canceled=False)
        handle.request_cancel()
        return CancelGoalResponse(canceled=True)

    def _publish_feedback(self, goal_id: str, feedback: Feedback) -> None:
        self._topics.publish(event_topic(self.name), ActionEvent(goal_id, feedback=feedback))

    def _run_goal(self, msg: GoalMessage) -> None:
        with self._lock:
            handle = self._goals[msg.goal_id]
        status, result = GoalStatus.CANCELED, None  # 시작 전에 취소됐으면 실행하지 않는다
        if not handle.is_cancel_requested:
            try:
                result = self._execute(handle)
                if not isinstance(result, self._type.result):
                    raise TypeError(
                        f"action {self.name!r} result is {self._type.result.__name__}, got {type(result).__name__}"
                    )
                status = GoalStatus.CANCELED if handle.is_cancel_requested else GoalStatus.SUCCEEDED
            except Exception:
                log.exception("action %r goal %s aborted", self.name, msg.goal_id)
                status, result = GoalStatus.ABORTED, None
        with self._lock:
            del self._goals[msg.goal_id]
        self._topics.publish(event_topic(self.name), ActionEvent(msg.goal_id, result=ResultMessage(msg.goal_id, status, result)))

    def destroy(self) -> None:
        self._sm.destroy_service(self._send)
        self._sm.destroy_service(self._cancel)
        self._topics.unsubscribe(self._goal_sub)  # 대기 중인 goal은 버려진다
        with self._lock:
            for handle in self._goals.values():
                handle.request_cancel()  # 실행 중인 goal은 취소 요청
