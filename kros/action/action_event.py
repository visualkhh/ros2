from dataclasses import dataclass
from typing import Any

from .result_message import ResultMessage


@dataclass(frozen=True)
class ActionEvent:
    """서버 -> 클라이언트 이벤트: feedback 또는 최종 result 중 하나.

    한 토픽(=구독 하나)으로 보내야 feedback이 result보다 늦게 처리되지 않는다 (순서 보장).
    """

    goal_id: str
    feedback: Any | None = None
    result: ResultMessage | None = None
