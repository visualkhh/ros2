from dataclasses import dataclass


@dataclass(frozen=True)
class CancelGoalRequest:
    goal_id: str
