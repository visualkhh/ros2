from dataclasses import dataclass, field

from kros.action.action_type import ActionType
from kros.msgs.geometry_msgs import Pose, PoseStamped
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class NavigateToPoseGoal:
    pose: PoseStamped
    behavior_tree: str = ""  # nav2와 같은 자리 (kros는 BT 파일 없이 고정된 흐름: 계획 -> 따라가기 -> 주기적 재계획)


@dataclass(frozen=True)
class NavigateToPoseFeedback:
    current_pose: PoseStamped = field(default_factory=lambda: PoseStamped(Header(0.0), Pose()))
    navigation_time: float = 0.0
    distance_remaining: float = 0.0
    number_of_recoveries: int = 0


@dataclass(frozen=True)
class NavigateToPoseResult:
    """nav2처럼 실패도 결과로: error_code는 이 코드이거나 planner(200번대)/controller(100번대)가 낸 코드."""

    NONE = 0
    UNKNOWN = 9000
    FAILED_TO_LOAD_BEHAVIOR_TREE = 9001
    TF_ERROR = 9002
    TIMEOUT = 9003

    error_code: int = 0
    error_msg: str = ""


NavigateToPose = ActionType("nav2_msgs/action/NavigateToPose", NavigateToPoseGoal, NavigateToPoseFeedback, NavigateToPoseResult)
