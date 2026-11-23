from dataclasses import dataclass, field

from kros.action.action_type import ActionType
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class BackUpGoal:
    backup_dist: float = 0.3  # 뒤로 물러설 거리 (m)
    backup_speed: float = 0.1  # 뒤로 물러설 속도 (m/s, 양수)
    time_allowance: float = 10.0


@dataclass(frozen=True)
class BackUpFeedback:
    distance_traveled: float = 0.0
    total_elapsed_time: float = 0.0


@dataclass(frozen=True)
class BackUpResult:
    """nav2 backup 행동의 결과."""

    NONE = 0
    UNKNOWN = 410
    TF_ERROR = 411
    TIMEOUT = 412
    COLLISION = 413

    error_code: int = 0
    error_msg: str = ""


BackUp = ActionType("nav2_msgs/action/BackUp", BackUpGoal, BackUpFeedback, BackUpResult)
