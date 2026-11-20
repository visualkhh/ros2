from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class GoalMessage:
    goal_id: str
    goal: Any
