from dataclasses import dataclass, field

from kros.action.action_type import ActionType
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class WaitGoal:
    wait_duration: float = 1.0  # 기다릴 시간 (s)


@dataclass(frozen=True)
class WaitFeedback:
    time_left: float = 0.0


@dataclass(frozen=True)
class WaitResult:
    """nav2 wait 행동의 결과."""

    NONE = 0
    UNKNOWN = 420

    error_code: int = 0
    error_msg: str = ""


Wait = ActionType("nav2_msgs/action/Wait", WaitGoal, WaitFeedback, WaitResult)
