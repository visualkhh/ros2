from dataclasses import dataclass
from typing import Generic, TypeVar

Goal = TypeVar("Goal")
Feedback = TypeVar("Feedback")
Result = TypeVar("Result")


@dataclass(frozen=True)
class ActionType(Generic[Goal, Feedback, Result]):
    """ROS .action 흉내: Goal / Feedback / Result 타입 세 개."""

    name: str
    goal: type[Goal]
    feedback: type[Feedback]
    result: type[Result]
