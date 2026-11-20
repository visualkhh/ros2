import math
import time
from collections.abc import Mapping

from kros.action.server_goal_handle import ServerGoalHandle
from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose, PoseStamped, Quaternion
from kros.msgs.nav2_msgs import ComputePathToPose, ComputePathToPoseFeedback, ComputePathToPoseGoal, ComputePathToPoseResult
from kros.msgs.nav_msgs import Path
from kros.msgs.std_msgs import Header
from kros.nodes.nav2.costmap_2d import Costmap2DROS
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.tf import TransformException

from .global_planner import GlobalPlanner, PlannerError
from .navfn_planner import NavfnPlanner
from .smac_planner_2d import SmacPlanner2D
from .smac_planner_hybrid import SmacPlannerHybrid
from .theta_star_planner import ThetaStarPlanner

PLUGINS: dict[str, type[GlobalPlanner]] = {  # nav2처럼 플러그인 이름으로 알고리즘을 고른다
    "nav2_navfn_planner::NavfnPlanner": NavfnPlanner,
    "nav2_theta_star_planner::ThetaStarPlanner": ThetaStarPlanner,
    "nav2_smac_planner::SmacPlanner2D": SmacPlanner2D,
    "nav2_smac_planner::SmacPlannerHybrid": SmacPlannerHybrid,
}
R = ComputePathToPoseResult


def _yaw(q: Quaternion) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _orientation(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class PlannerServer(Node):
    """nav2 planner_server 흉내: compute_path_to_pose 액션(nav2_msgs/ComputePathToPose)을 받아 경로를 계획하고 plan 토픽(nav_msgs/Path)으로도 낸다.
    nav2처럼 자기 안에 global_costmap(Costmap2DROS) 노드를 만든다 (그 파라미터는 costmap_parameters로).

    파라미터: planner_plugins (플러그인 이름들, 기본 ("GridBased",)), "<이름>.plugin" (알고리즘: PLUGINS의 키), 그 밖 "<이름>.<값>"은 알고리즘마다.
    예: {"planner_plugins": ("GridBased", "Theta"), "GridBased.use_astar": True, "Theta.plugin": "nav2_theta_star_planner::ThetaStarPlanner"}
    goal.planner_id로 플러그인을 고르고, 비면 첫 플러그인이다.
    """

    def __init__(
        self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None, costmap_parameters: Mapping[str, ParameterValue] | None = None
    ) -> None:
        super().__init__(context, name or "planner_server", parameters)
        self.costmap_ros = Costmap2DROS(context, "global_costmap", costmap_parameters)
        self.planner_plugins = self.declare_parameter("planner_plugins", ("GridBased",), validator=lambda ids: len(ids) > 0)
        self.planners: dict[str, GlobalPlanner] = {}
        for plugin_id in self.planner_plugins.value:
            kind = self.declare_parameter(f"{plugin_id}.plugin", "nav2_navfn_planner::NavfnPlanner", validator=lambda value: value in PLUGINS)
            planner = PLUGINS[kind.value]()
            planner.configure(self, str(plugin_id))
            self.planners[str(plugin_id)] = planner
        self.create_action_server("compute_path_to_pose", ComputePathToPose, self._execute)

    def _execute(self, handle: ServerGoalHandle[ComputePathToPoseGoal, ComputePathToPoseFeedback]) -> ComputePathToPoseResult:
        began = time.perf_counter()
        goal = handle.goal
        result = self.compute_path(goal)
        return R(result.path, time.perf_counter() - began, result.error_code, result.error_msg)

    def compute_path(self, goal: ComputePathToPoseGoal) -> ComputePathToPoseResult:
        planner_id = goal.planner_id or str(self.planner_plugins.value[0])
        planner = self.planners.get(planner_id)
        if planner is None:
            return self._fail(R.INVALID_PLANNER, f"플래너 {planner_id!r}가 없다 (있는 것: {', '.join(self.planners)})")
        costmap = self.costmap_ros.costmap
        if costmap is None:
            return self._fail(R.UNKNOWN, "global_costmap에 지도가 아직 없다 (map_server/SLAM)")
        frame = self.costmap_ros.global_frame.value
        try:
            start = self._in_frame(goal.start, frame) if goal.use_start else self.costmap_ros.get_robot_pose()
            target = self._in_frame(goal.goal, frame)
        except TransformException as error:
            return self._fail(R.TF_ERROR, str(error))
        if start is None:
            return self._fail(R.TF_ERROR, f"{frame} -> {self.costmap_ros.robot_base_frame.value} TF가 없어 로봇 위치를 모른다")
        try:
            points = planner.create_plan(costmap, start, target)
        except PlannerError as error:
            return self._fail(error.code, str(error))
        points[-1] = target[:2]  # 마지막 점은 칸 중심이 아니라 목표 그대로 (tolerance로 옮겼으면 옮긴 칸 그대로 둔다)
        stamp = self.now()
        poses = []
        for k, (x, y) in enumerate(points):
            if k + 1 < len(points):
                yaw = math.atan2(points[k + 1][1] - y, points[k + 1][0] - x)  # 다음 점을 본다
            else:
                yaw = target[2]  # 끝은 목표 방향 (nav2와 같다)
            poses.append(PoseStamped(Header(stamp, frame), Pose(Point(x, y, 0.0), _orientation(yaw))))
        path = Path(Header(stamp, frame), tuple(poses))
        self.publish("plan", path)
        return R(path)

    def _in_frame(self, pose: PoseStamped, frame: str) -> tuple[float, float, float]:
        p, yaw = pose.pose.position, _yaw(pose.pose.orientation)
        if pose.header.frame_id in ("", frame):
            return p.x, p.y, yaw
        t = self.costmap_ros.buffer.lookup_transform(frame, pose.header.frame_id).transform
        ty = _yaw(t.rotation)
        c, s = math.cos(ty), math.sin(ty)
        return t.translation.x + c * p.x - s * p.y, t.translation.y + s * p.x + c * p.y, ty + yaw

    def _fail(self, code: int, message: str) -> ComputePathToPoseResult:
        self.get_logger().warn("경로 계획 실패 (%d): %s", code, message)
        return R(Path(Header(self.now(), self.costmap_ros.global_frame.value)), 0.0, code, message)
