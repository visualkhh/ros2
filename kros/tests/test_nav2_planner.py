import math
import unittest

from kros import Kros
from kros.action import GoalStatus
from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Vector3
from kros.msgs.nav2_msgs import ComputePathToPose, ComputePathToPoseGoal, ComputePathToPoseResult
from kros.msgs.nav_msgs import FREE, OCCUPIED, MapMetaData, OccupancyGrid, Path
from kros.msgs.std_msgs import Header
from kros.nodes import Node
from kros.nodes.nav2.costmap_2d import INSCRIBED_INFLATED_OBSTACLE, Costmap
from kros.nodes.nav2.planner import NavfnPlanner, PlannerError, PlannerServer, SmacPlanner2D, SmacPlannerHybrid, ThetaStarPlanner
from kros.nodes.nav2.planner.global_planner import GlobalPlanner
from kros.parameter import ParameterValue
from kros.tf import TransformBroadcaster
from kros.timer import SimulatedTimeSource
from kros.topic import QOS_LATCHED

RES, W, H = 0.05, 60, 40  # 3 x 2 m 방


def room(gap: bool = True) -> OccupancyGrid:
    """가운데(x = 1.5m)에 세로 벽. gap이면 위쪽 0.5m(y 1.5~2.0)가 뚫려 있다."""
    data = [FREE] * (W * H)
    for j in range(H):
        for i in range(W):
            if i in (0, W - 1) or j in (0, H - 1) or (i == W // 2 and (j < 30 or not gap)):
                data[j * W + i] = OCCUPIED
    return OccupancyGrid(Header(0.0, "map"), MapMetaData(0.0, RES, W, H), tuple(data))


def costmap(gap: bool = True) -> Costmap:
    made = Costmap.from_occupancy_grid(room(gap))
    made.inflate(0.1, 0.3, 3.0)
    return made


def plugin(kind: type[GlobalPlanner], **parameters: bool | float) -> GlobalPlanner:
    """노드 없이 시험하려고: 파라미터만 가진 노드에 플러그인을 붙인다."""
    made: dict[str, Node] = {}
    kros = Kros(lambda ctx: [made.setdefault("n", Node(ctx, "planner", parameters={f"p.{k}": v for k, v in parameters.items()}))], time_source=SimulatedTimeSource())
    unittest.addModuleCleanup(kros.shutdown)
    planner = kind()
    planner.configure(made["n"], "p")
    return planner


def length(points: list[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def min_radius(points: list[tuple[float, float]]) -> float:
    """연속한 세 점의 외접원 반지름 중 가장 작은 것 (경로가 가장 급하게 도는 곳)."""
    radius = math.inf
    for a, b, c in zip(points, points[1:], points[2:]):
        area = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2
        if area > 1e-9:
            radius = min(radius, math.dist(a, b) * math.dist(b, c) * math.dist(c, a) / (4 * area))
    return radius


def turns(points: list[tuple[float, float]]) -> int:
    """방향이 바뀌는 횟수."""
    headings = [round(math.atan2(b[1] - a[1], b[0] - a[0]), 3) for a, b in zip(points, points[1:]) if a != b]
    return sum(1 for a, b in zip(headings, headings[1:]) if a != b)


class TestPlanners(unittest.TestCase):
    START, GOAL = (0.5, 0.5, 0.0), (2.5, 0.5, 0.0)  # 벽 왼쪽 아래 -> 오른쪽 아래: 위쪽 틈으로 돌아가야 한다

    def check_path(self, grid: Costmap, points: list[tuple[float, float]]) -> None:
        self.assertLess(math.dist(points[0], self.START[:2]), RES)
        self.assertLess(math.dist(points[-1], self.GOAL[:2]), RES)
        self.assertGreater(max(y for _, y in points), 1.5)  # 틈으로 갔다
        for x, y in points:
            self.assertLess(grid.cost_at(x, y), INSCRIBED_INFLATED_OBSTACLE, (x, y))  # 몸이 벽에 닿는 곳을 지나지 않는다
        for a, b in zip(points, points[1:]):
            self.assertLessEqual(math.dist(a, b), RES * 1.5)  # 끊기지 않는다

    def test_dijkstra_and_astar_find_the_same_length_but_astar_looks_at_less(self) -> None:
        grid = costmap()
        dijkstra, astar = plugin(NavfnPlanner), plugin(NavfnPlanner, use_astar=True)
        assert isinstance(dijkstra, NavfnPlanner) and isinstance(astar, NavfnPlanner)
        a, b = dijkstra.create_plan(grid, self.START, self.GOAL), astar.create_plan(grid, self.START, self.GOAL)
        self.check_path(grid, a)
        self.check_path(grid, b)
        self.assertAlmostEqual(length(a), length(b), delta=0.15)
        self.assertLess(astar.expanded, dijkstra.expanded)

    def test_theta_star_is_shorter_and_straighter(self) -> None:
        grid = costmap()
        theta = plugin(ThetaStarPlanner, w_traversal_cost=0.0)
        navfn = plugin(NavfnPlanner)
        a, b = theta.create_plan(grid, self.START, self.GOAL), navfn.create_plan(grid, self.START, self.GOAL)
        self.check_path(grid, a)
        self.assertLess(length(a), length(b))  # 45도 단위로 꺾이지 않고 질러간다
        self.assertLess(turns(a), turns(b))

    def test_theta_star_goes_straight_at_any_angle_in_open_space(self) -> None:
        grid = costmap()
        start, goal = (0.33, 0.33, 0.0), (1.23, 0.78, 0.0)  # 막힘 없는 왼쪽 방, 26도쯤: 8방향 격자로는 한 번 꺾어야 한다
        straight = plugin(ThetaStarPlanner, w_traversal_cost=0.0).create_plan(grid, start, goal)
        self.assertEqual(turns(straight), 0)
        self.assertGreater(turns(plugin(NavfnPlanner, use_astar=True).create_plan(grid, start, goal)), 0)

    def test_smac_2d_smoother_rounds_the_corners(self) -> None:
        grid = costmap()
        smooth = plugin(SmacPlanner2D).create_plan(grid, self.START, self.GOAL)
        raw = plugin(SmacPlanner2D, smooth_path=False).create_plan(grid, self.START, self.GOAL)
        self.check_path(grid, smooth)
        self.assertGreater(min_radius(smooth), min_radius(raw) * 1.2)  # 꺾인 칸 경로를 편다

    def test_smac_hybrid_respects_the_turning_radius_and_start_heading(self) -> None:
        grid = costmap()
        for heading in (0.0, math.pi / 2):
            points = plugin(SmacPlannerHybrid, minimum_turning_radius=0.3).create_plan(grid, (0.5, 0.5, heading), self.GOAL)
            self.check_path(grid, points)
            self.assertGreater(min_radius(points), 0.3 * 0.95)  # 로봇이 달릴 수 있는 곡률
            first = math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0])
            self.assertAlmostEqual(first, heading, delta=0.2)  # 지금 보는 방향으로 출발한다

    def test_failures_have_nav2_error_codes(self) -> None:
        navfn = plugin(NavfnPlanner, tolerance=0.0)
        with self.assertRaises(PlannerError) as caught:
            navfn.create_plan(costmap(gap=False), self.START, self.GOAL)
        self.assertEqual(caught.exception.code, ComputePathToPoseResult.NO_VALID_PATH)
        for goal, code in (((9.0, 0.5, 0.0), ComputePathToPoseResult.GOAL_OUTSIDE_MAP), ((1.52, 0.5, 0.0), ComputePathToPoseResult.GOAL_OCCUPIED)):
            with self.assertRaises(PlannerError) as caught:
                navfn.create_plan(costmap(), self.START, goal)
            self.assertEqual(caught.exception.code, code)

    def test_goal_in_a_wall_moves_within_tolerance_and_a_robot_against_a_wall_can_leave(self) -> None:
        grid = costmap()
        points = plugin(NavfnPlanner, tolerance=0.5).create_plan(grid, self.START, (1.52, 0.5, 0.0))  # 벽 속 목표
        self.assertLess(math.dist(points[-1], (1.52, 0.5)), 0.5)
        self.assertLess(grid.cost_at(*points[-1]), INSCRIBED_INFLATED_OBSTACLE)
        hugging = (1.43, 0.5, 0.0)  # 벽에서 0.07m: 몸이 닿는 칸에 서 있다
        self.assertGreaterEqual(grid.cost_at(*hugging[:2]), INSCRIBED_INFLATED_OBSTACLE)
        self.assertTrue(plugin(NavfnPlanner).create_plan(grid, hugging, self.START))


class TestPlannerServer(unittest.TestCase):
    def setUp(self) -> None:
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["user"] = Node(ctx, "User")
            parameters: dict[str, ParameterValue] = {"planner_plugins": ("GridBased", "Theta"), "Theta.plugin": "nav2_theta_star_planner::ThetaStarPlanner"}
            made["server"] = PlannerServer(ctx, parameters=parameters, costmap_parameters={"robot_radius": 0.1, "inflation_radius": 0.3})
            return list(made.values())

        self.kros = Kros(create, time_source=SimulatedTimeSource())
        self.addCleanup(self.kros.shutdown)
        self.user = made["user"]
        self.plans: list[Path] = []
        self.user.subscribe("plan", Path, self.plans.append)
        self.client = self.user.create_action_client("compute_path_to_pose", ComputePathToPose)

    def ready(self) -> None:
        self.user.publish("map", room(), QOS_LATCHED)
        TransformBroadcaster(self.user).send_transform(TransformStamped(Header(0.0, "map"), "base_footprint", Transform(Vector3(0.5, 0.5, 0.0), Quaternion())))
        self.kros.context.topic_manager.join()

    def plan(self, planner_id: str = "") -> ComputePathToPoseResult:
        goal = PoseStamped(Header(0.0, "map"), Pose(Point(2.5, 0.5, 0.0), Quaternion(0.0, 0.0, math.sin(0.5), math.cos(0.5))))
        message = self.client.send_goal(ComputePathToPoseGoal(goal, planner_id=planner_id), timeout=5).result(10)
        self.assertEqual(message.status, GoalStatus.SUCCEEDED)
        assert isinstance(message.result, ComputePathToPoseResult)
        return message.result

    def test_plans_from_the_robot_pose_to_the_goal_and_publishes_plan(self) -> None:
        self.ready()
        result = self.plan()
        self.assertEqual(result.error_code, ComputePathToPoseResult.NONE)
        poses = result.path.poses
        self.assertEqual((poses[0].pose.position.x, poses[0].pose.position.y), (0.525, 0.525))  # 로봇이 있는 칸
        self.assertEqual((poses[-1].pose.position.x, poses[-1].pose.position.y), (2.5, 0.5))  # 목표 그대로
        last = poses[-1].pose.orientation
        self.assertAlmostEqual(2 * math.atan2(last.z, last.w), 1.0)  # 끝은 목표 방향
        self.assertEqual(result.path.header.frame_id, "map")
        self.kros.context.topic_manager.join()
        self.assertEqual(self.plans[-1], result.path)

    def test_planner_id_picks_the_algorithm(self) -> None:
        self.ready()
        navfn, theta = self.plan("GridBased"), self.plan("Theta")
        self.assertLess(len(theta.path.poses), len(navfn.path.poses) + 5)
        self.assertEqual(self.plan("Nope").error_code, ComputePathToPoseResult.INVALID_PLANNER)

    def test_without_map_or_tf_it_reports_why(self) -> None:
        with self.assertLogs("kros.node.planner_server", level="WARNING"):
            self.assertEqual(self.plan().error_code, ComputePathToPoseResult.UNKNOWN)  # 지도 없음
        self.user.publish("map", room(), QOS_LATCHED)
        self.kros.context.topic_manager.join()
        with self.assertLogs("kros.node.planner_server", level="WARNING"):
            self.assertEqual(self.plan().error_code, ComputePathToPoseResult.TF_ERROR)  # 로봇 위치 모름


if __name__ == "__main__":
    unittest.main()
