import threading
import uuid
from typing import Any, Callable, Generic, TypeVar

from kros.topic import TopicExecutor
from kros.service import ServiceExecutor

from .action_type import ActionType
from .cancel_goal_request import CancelGoalRequest
from .client_goal_handle import ClientGoalHandle
from .action_event import ActionEvent
from .goal_status import GoalStatus
from .result_message import ResultMessage
from .send_goal_request import SendGoalRequest
from .wire import CANCEL_GOAL, SEND_GOAL, cancel_goal_service, event_topic, send_goal_service

Goal = TypeVar("Goal")
Feedback = TypeVar("Feedback")
Result = TypeVar("Result")


class ActionClient(Generic[Goal, Feedback, Result]):
    """액션 클라이언트. send_goal은 서버가 받을지(accepted)만 기다리고, 결과는 핸들로 따로 기다린다."""

    def __init__(
        self,
        name: str,
        action_type: ActionType[Goal, Feedback, Result],
        topic_executor: TopicExecutor,
        service_executor: ServiceExecutor,
    ) -> None:
        self.name = name
        self._type = action_type
        self._topics = topic_executor
        self._sm = service_executor
        self._lock = threading.Lock()
        self._handles: dict[str, ClientGoalHandle] = {}
        self._event_sub = topic_executor.subscribe(event_topic(name), ActionEvent, self._on_event)

    def send_goal(
        self, goal: Goal, feedback_callback: Callable[[Feedback], None] | None = None, timeout: float | None = None
    ) -> ClientGoalHandle:
        if not isinstance(goal, self._type.goal):
            raise TypeError(f"action {self.name!r} goal is {self._type.goal.__name__}, got {type(goal).__name__}")
        goal_id = uuid.uuid4().hex
        handle = ClientGoalHandle(goal_id, self._cancel, feedback_callback)
        with self._lock:  # 응답보다 feedback/result가 먼저 올 수 있어 보내기 전에 등록
            self._handles[goal_id] = handle
        try:
            response = self._sm.call(send_goal_service(self.name), SEND_GOAL, SendGoalRequest(goal_id, goal), timeout)
        except BaseException:
            self._forget(goal_id)
            raise
        if not response.accepted:
            self._forget(goal_id)
            handle.finish(ResultMessage(goal_id, GoalStatus.REJECTED, None))
        return handle

    def _cancel(self, goal_id: str, timeout: float | None) -> bool:
        response = self._sm.call(cancel_goal_service(self.name), CANCEL_GOAL, CancelGoalRequest(goal_id), timeout)
        return response.canceled

    def _forget(self, goal_id: str) -> ClientGoalHandle | None:
        with self._lock:
            return self._handles.pop(goal_id, None)

    def _on_event(self, event: ActionEvent) -> None:
        if event.result is not None:
            handle = self._forget(event.goal_id)  # 다른 클라이언트의 goal이면 None
            if handle is not None:
                handle.finish(event.result)
            return
        with self._lock:
            handle = self._handles.get(event.goal_id)
        if handle is not None and handle.feedback_callback is not None:
            handle.feedback_callback(event.feedback)

    def destroy(self) -> None:
        """클라이언트를 내린다. 결과를 기다리던 핸들은 영원히 기다리지 않게 ABORTED로 끝낸다 (여러 번 불러도 안전)."""
        self._topics.unsubscribe(self._event_sub)
        with self._lock:
            pending = list(self._handles.items())
            self._handles.clear()
        for goal_id, handle in pending:
            handle.finish(ResultMessage(goal_id, GoalStatus.ABORTED, None))
