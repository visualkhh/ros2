import threading
from typing import Any, Callable, Generic, TypeVar

from .action_type import ActionType

Goal = TypeVar("Goal")
Feedback = TypeVar("Feedback")


class ServerGoalHandle(Generic[Goal, Feedback]):
    """서버의 execute 콜백이 받는 핸들: goal, 취소 여부, feedback 발행."""

    def __init__(
        self,
        goal_id: str,
        goal: Goal,
        action_type: ActionType[Goal, Feedback, Any],
        publish: Callable[[str, Feedback], None],
    ) -> None:
        self.goal_id: str = goal_id
        self.goal: Goal = goal
        self._action_type = action_type
        self._publish = publish
        self._cancel = threading.Event()

    @property
    def is_cancel_requested(self) -> bool:
        return self._cancel.is_set()

    def request_cancel(self) -> None:
        self._cancel.set()

    def publish_feedback(self, feedback: Feedback) -> None:
        if not isinstance(feedback, self._action_type.feedback):
            raise TypeError(
                f"action {self._action_type.name!r} feedback is {self._action_type.feedback.__name__}, "
                f"got {type(feedback).__name__}"
            )
        self._publish(self.goal_id, feedback)
