from concurrent.futures import Future
from typing import Any, Callable

from .goal_status import GoalStatus
from .result_message import ResultMessage


class ClientGoalHandle:
    """보낸 goal 하나에 대한 클라이언트 쪽 핸들: 상태, 결과 기다리기, 취소."""

    def __init__(
        self,
        goal_id: str,
        cancel: Callable[[str, float | None], bool],
        feedback_callback: Callable[[Any], None] | None,
    ) -> None:
        self.goal_id: str = goal_id
        self.status: GoalStatus = GoalStatus.ACCEPTED
        self.feedback_callback = feedback_callback
        self._cancel = cancel
        self._future: Future[ResultMessage] = Future()

    @property
    def accepted(self) -> bool:
        return self.status is not GoalStatus.REJECTED

    def result(self, timeout: float | None = None) -> ResultMessage:
        return self._future.result(timeout)

    def cancel(self, timeout: float | None = None) -> bool:
        """취소 요청을 보낸다. 서버가 요청을 받았으면 True (실제 종료는 result()로 확인)."""
        return self._cancel(self.goal_id, timeout)

    def finish(self, msg: ResultMessage) -> None:
        self.status = msg.status
        self._future.set_result(msg)
