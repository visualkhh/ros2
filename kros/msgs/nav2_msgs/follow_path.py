from dataclasses import dataclass, field

from kros.action.action_type import ActionType
from kros.msgs.nav_msgs import Path
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class FollowPathGoal:
    path: Path = field(default_factory=lambda: Path(Header(0.0)))
    controller_id: str = ""  # 비면 controller_server의 첫 플러그인
    goal_checker_id: str = ""


@dataclass(frozen=True)
class FollowPathFeedback:
    distance_to_goal: float = 0.0
    speed: float = 0.0


@dataclass(frozen=True)
class FollowPathResult:
    """nav2처럼 실패도 결과로 돌려준다 (error_code가 NONE이 아니면 실패)."""

    NONE = 0
    UNKNOWN = 100
    INVALID_CONTROLLER = 101
    TF_ERROR = 102
    INVALID_PATH = 103
    PATIENCE_EXCEEDED = 104
    FAILED_TO_MAKE_PROGRESS = 105
    NO_VALID_CONTROL = 106

    error_code: int = 0
    error_msg: str = ""


FollowPath = ActionType("nav2_msgs/action/FollowPath", FollowPathGoal, FollowPathFeedback, FollowPathResult)
