from dataclasses import dataclass, field

from kros.action.action_type import ActionType
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class SpinGoal:
    target_yaw: float = 3.141592653589793  # 상대 회전 각 (rad)
    time_allowance: float = 10.0


@dataclass(frozen=True)
class SpinFeedback:
    angular_distance_traveled: float = 0.0
    total_elapsed_time: float = 0.0


@dataclass(frozen=True)
class SpinResult:
    """nav2 spin 행동의 결과."""

    NONE = 0
    UNKNOWN = 400
    TF_ERROR = 401
    TIMEOUT = 402

    error_code: int = 0
    error_msg: str = ""


Spin = ActionType("nav2_msgs/action/Spin", SpinGoal, SpinFeedback, SpinResult)
