from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SendGoalRequest:
    goal_id: str
    goal: Any
