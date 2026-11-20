from dataclasses import dataclass
from typing import Any

from .goal_status import GoalStatus


@dataclass(frozen=True)
class ResultMessage:
    goal_id: str
    status: GoalStatus
    result: Any | None  # SUCCEEDED/CANCELED(결과를 돌려줬을 때)만 값이 있다
