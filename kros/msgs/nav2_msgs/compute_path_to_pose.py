from dataclasses import dataclass, field

from kros.action.action_type import ActionType
from kros.msgs.geometry_msgs import Pose, PoseStamped
from kros.msgs.nav_msgs import Path
from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class ComputePathToPoseGoal:
    goal: PoseStamped
    start: PoseStamped = field(default_factory=lambda: PoseStamped(Header(0.0), Pose()))
    planner_id: str = ""  # 비면 planner_server의 첫 플러그인
    use_start: bool = False  # False면 로봇의 지금 위치에서 시작


@dataclass(frozen=True)
class ComputePathToPoseFeedback:
    pass


@dataclass(frozen=True)
class ComputePathToPoseResult:
    """nav2처럼 실패도 결과로 돌려준다: error_code가 NONE이 아니면 path는 비어 있다."""

    NONE = 0
    UNKNOWN = 200
    INVALID_PLANNER = 201
    TF_ERROR = 202
    START_OUTSIDE_MAP = 203
    GOAL_OUTSIDE_MAP = 204
    START_OCCUPIED = 205
    GOAL_OCCUPIED = 206
    TIMEOUT = 207
    NO_VALID_PATH = 208

    path: Path = field(default_factory=lambda: Path(Header(0.0)))
    planning_time: float = 0.0
    error_code: int = 0
    error_msg: str = ""


ComputePathToPose = ActionType("nav2_msgs/action/ComputePathToPose", ComputePathToPoseGoal, ComputePathToPoseFeedback, ComputePathToPoseResult)
