import math
import time
import unittest
from collections.abc import Callable

from kros import Kros
from kros.action import GoalStatus
from kros.context import Context
from kros.examples.turtlebot3.main import IMU_CONFIG, ODOM_CONFIG
from kros.examples.turtlebot3.robot_bridge import RobotBridge
from kros.msgs.geometry_msgs import Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Twist, Vector3
from kros.msgs.nav2_msgs import ComputePathToPose, ComputePathToPoseGoal, ComputePathToPoseResult, FollowPath, FollowPathGoal, FollowPathResult
from kros.msgs.nav_msgs import FREE, OCCUPIED, MapMetaData, OccupancyGrid, Path
from kros.msgs.std_msgs import Header
from kros.nodes import Cartographer, DiffDriveOdometry, Ekf, Node, RobotStatePublisher
from kros.nodes.nav2.controller import ControllerError, ControllerServer, DWBLocalPlanner, MPPIController, RegulatedPurePursuitController, SimpleGoalChecker, SimpleProgressChecker
from kros.nodes.nav2.controller.controller import Controller, Pose2, blocked
from kros.nodes.nav2.costmap_2d import Costmap
from kros.nodes.nav2.planner import PlannerServer
from kros.parameter import ParameterValue
from kros.tf import TransformBroadcaster
from kros.timer import SimulatedTimeSource
from kros.topic import QOS_LATCHED
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3

RES, W, H = 0.05, 80, 80  # 4 x 4 m, 가운데가 원점


def room(walls: list[tuple[float, float]] = []) -> OccupancyGrid:
    """둘레가 벽인 방 (origin (-2, -2)). walls: 더 칠할 벽 칸의 world 좌표."""
    data = [OCCUPIED if i in (0, W - 1) or j in (0, H - 1) else FREE for j in range(H) for i in range(W)]
    for x, y in walls:
        data[int((y + 2) / RES) * W + int((x + 2) / RES)] = OCCUPIED
    return OccupancyGrid(Header(0.0, "map"), MapMetaData(0.0, RES, W, H, Pose(Point(-2.0, -2.0, 0.0))), tuple(data))


def costmap(walls: list[tuple[float, float]] = []) -> Costmap:
    made = Costmap.from_occupancy_grid(room(walls))
    made.inflate(0.2, 0.5, 3.0)
    return made


def line(start: tuple[float, float], end: tuple[float, float], yaw: float | None = None) -> list[Pose2]:
    heading = math.atan2(end[1] - start[1], end[0] - start[0])
    n = max(1, round(math.dist(start, end) / RES))
    points = [(start[0] + (end[0] - start[0]) * k / n, start[1] + (end[1] - start[1]) * k / n, heading) for k in range(n + 1)]
    points[-1] = (*points[-1][:2], heading if yaw is None else yaw)
    return points


class Plugins:
    def __init__(self, test: unittest.TestCase, **parameters: float | bool) -> None:
        made: dict[str, Node] = {}
        self.kros = Kros(lambda ctx: [made.setdefault("n", Node(ctx, "controller", parameters={f"c.{k}": v for k, v in parameters.items()}))], time_source=SimulatedTimeSource())
        test.addCleanup(self.kros.shutdown)
        self.node = made["n"]

    def make(self, kind: type[Controller]) -> Controller:
        controller = kind()
        controller.configure(self.node, "c")
        return controller


class TestRegulatedPurePursuit(unittest.TestCase):
    def setUp(self) -> None:
        self.rpp = Plugins(self).make(RegulatedPurePursuitController)
        self.grid = costmap()

    def test_straight_path_goes_straight_at_the_desired_speed(self) -> None:
        v, w = self.rpp.compute_velocity_commands((0.0, 0.0, 0.0), (0.22, 0.0), line((0.0, 0.0), (1.5, 0.0)), self.grid, 0.05)
        self.assertAlmostEqual(v, 0.22)
        self.assertAlmostEqual(w, 0.0)

    def test_path_behind_rotates_in_place_first(self) -> None:
        v, w = self.rpp.compute_velocity_commands((0.0, 0.0, 0.0), (0.0, 0.0), line((0.0, 0.0), (0.0, 1.5)), self.grid, 0.05)  # 왼쪽 90도
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)
        self.assertLessEqual(w, 3.2 * 0.05 + 1e-9)  # 가속 한계 (정지에서 한 주기)

    def test_tight_curve_slows_down(self) -> None:
        arc = [(0.3 * math.sin(t / 20), 0.3 - 0.3 * math.cos(t / 20), t / 20) for t in range(40)]  # 반지름 0.3m 왼쪽 원호
        v, w = self.rpp.compute_velocity_commands((0.0, 0.0, 0.0), (0.2, 0.0), arc, self.grid, 0.05)
        self.assertLess(v, 0.22 * 0.6)  # 반지름 0.3 < 0.9: 늦춘다
        self.assertGreater(w, 0.0)

    def test_near_the_goal_only_turns_to_its_heading(self) -> None:
        v, w = self.rpp.compute_velocity_commands((1.4, 0.0, 0.0), (0.0, 0.0), line((0.0, 0.0), (1.5, 0.0), yaw=-1.0), self.grid, 0.05)
        self.assertEqual(v, 0.0)
        self.assertLess(w, 0.0)

    def test_wall_on_the_way_is_reported(self) -> None:
        grid = costmap([(0.4, y / 100) for y in range(-10, 11)])  # 0.4m 앞 벽
        with self.assertRaises(ControllerError) as caught:
            self.rpp.compute_velocity_commands((0.0, 0.0, 0.0), (0.22, 0.0), line((0.0, 0.0), (1.5, 0.0)), grid, 0.05)
        self.assertEqual(caught.exception.code, FollowPathResult.NO_VALID_CONTROL)


class TestDwb(unittest.TestCase):
    def setUp(self) -> None:
        dwb = Plugins(self).make(DWBLocalPlanner)
        assert isinstance(dwb, DWBLocalPlanner)
        self.dwb = dwb

    def test_follows_a_straight_path_and_speeds_up_within_the_acceleration_limit(self) -> None:
        v, w = self.dwb.compute_velocity_commands((0.0, 0.0, 0.0), (0.0, 0.0), line((0.0, 0.0), (1.5, 0.0)), costmap(), 0.05)
        self.assertAlmostEqual(v, 2.5 * 0.05)  # 서 있다가: 한 주기에 가속 한계만큼
        self.assertAlmostEqual(w, 0.0, places=6)
        self.assertTrue(self.dwb.best)

    def test_turns_toward_a_path_on_the_left(self) -> None:
        _, w = self.dwb.compute_velocity_commands((0.0, 0.0, 0.0), (0.1, 0.0), line((0.0, 0.0), (0.6, 0.6)), costmap(), 0.05)
        self.assertGreater(w, 0.0)

    def test_never_picks_a_trajectory_into_a_wall(self) -> None:
        grid = costmap([(0.45, y / 100) for y in range(-60, 61)])  # 경로를 가로막는 0.45m 앞 벽
        v, _ = self.dwb.compute_velocity_commands((0.0, 0.0, 0.0), (0.2, 0.0), line((0.0, 0.0), (1.5, 0.0)), grid, 0.05)
        self.assertTrue(all(not blocked(grid, x, y) for x, y, _ in self.dwb.best))  # 고른 자취는 벽에 닿지 않는다
        self.assertLess(v, 0.2)  # 그대로 달리면 1.7초 안에 닿는다: 늦춘다

    def test_at_the_goal_it_stops_and_turns(self) -> None:
        v, w = self.dwb.compute_velocity_commands((1.45, 0.0, 0.0), (0.0, 0.0), line((0.0, 0.0), (1.5, 0.0), yaw=1.0), costmap(), 0.05)
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)


class TestMppi(unittest.TestCase):
    def make(self, **parameters: float | bool) -> MPPIController:
        mppi = Plugins(self, **parameters).make(MPPIController)
        assert isinstance(mppi, MPPIController)
        return mppi

    def test_drives_forward_along_a_straight_path(self) -> None:
        mppi = self.make()
        path = line((0.0, 0.0), (1.5, 0.0))
        for _ in range(10):  # warm start: 몇 주기 지나면 시퀀스가 경로를 따른다
            v, w = mppi.compute_velocity_commands((0.0, 0.0, 0.0), (0.1, 0.0), path, costmap(), 0.05)
        self.assertGreater(v, 0.1)
        self.assertLess(abs(w), 0.3)
        self.assertGreater(mppi.best[-1][0], 0.2)  # 고른 자취가 앞으로 뻗는다 (2초 x 최고 0.22m/s 안)

    def test_turns_toward_a_path_behind(self) -> None:
        mppi = self.make()
        for _ in range(10):
            _, w = mppi.compute_velocity_commands((0.0, 0.0, 0.0), (0.0, 0.0), line((0.0, 0.0), (0.0, 1.5)), costmap(), 0.05)
        self.assertGreater(w, 0.3)  # 왼쪽 경로를 향해 돈다

    def test_same_seed_same_command_and_walls_all_around_fail(self) -> None:
        a, b = self.make(seed=3), self.make(seed=3)
        path = line((0.0, 0.0), (1.5, 0.0))
        self.assertEqual(a.compute_velocity_commands((0.0, 0.0, 0.0), (0.1, 0.0), path, costmap(), 0.05), b.compute_velocity_commands((0.0, 0.0, 0.0), (0.1, 0.0), path, costmap(), 0.05))
        walls = [(x / 100, y / 100) for x in range(-30, 31) for y in range(-30, 31) if max(abs(x), abs(y)) >= 25]
        with self.assertRaises(ControllerError):  # 0.25m 상자 (몸 반지름 0.2 + 잡음 자취는 모두 닿는다)
            self.make(vx_min=0.0, wz_max=0.01).compute_velocity_commands((0.0, 0.0, 0.0), (0.2, 0.0), path, costmap(walls), 0.05)


class TestCheckers(unittest.TestCase):
    def test_goal_and_progress_checkers(self) -> None:
        node = Plugins(self).node
        goal, progress = SimpleGoalChecker(node), SimpleProgressChecker(node)
        self.assertTrue(goal.is_goal_reached((0.9, 0.1, 0.1), (1.0, 0.0, 0.0)))
        self.assertFalse(goal.is_goal_reached((0.9, 0.1, 0.5), (1.0, 0.0, 0.0)))  # 방향이 틀렸다
        self.assertFalse(goal.is_goal_reached((0.6, 0.0, 0.0), (1.0, 0.0, 0.0)))
        self.assertTrue(progress.check((0.0, 0.0, 0.0), 0.0))
        self.assertTrue(progress.check((0.1, 0.0, 0.0), 9.0))
        self.assertFalse(progress.check((0.2, 0.0, 0.0), 10.5))  # 10초 넘게 0.5m를 못 갔다
        self.assertTrue(progress.check((0.7, 0.0, 0.0), 11.0))  # 움직이면 다시 센다


class TestControllerServer(unittest.TestCase):
    def setUp(self) -> None:
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["user"] = Node(ctx, "User")
            made["server"] = ControllerServer(ctx, costmap_parameters={"robot_radius": 0.2, "global_frame": "map"})  # 이 시험의 TF는 map -> base뿐
            return list(made.values())

        self.source = SimulatedTimeSource()
        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        self.user = made["user"]
        self.commands: list[Twist] = []
        self.user.subscribe("cmd_vel", Twist, self.commands.append)
        self.client = self.user.create_action_client("follow_path", FollowPath)
        self.user.publish("map", room(), QOS_LATCHED)
        TransformBroadcaster(self.user).send_transform(TransformStamped(Header(0.0, "map"), "base_footprint", Transform(Vector3(), Quaternion())))
        self.kros.context.topic_executor.join()

    def path(self, frame: str = "map") -> Path:
        return Path(Header(0.0, frame), tuple(PoseStamped(Header(0.0, frame), Pose(Point(x, y, 0.0), Quaternion())) for x, y, _ in line((0.0, 0.0), (1.5, 0.0))))

    def run_until(self, done: Callable[[], bool]) -> None:
        """시뮬레이션 시간을 제어 주기씩 민다. goal이 도는 동안은 topic_executor.join()을 부르지 않는다 (goal 실행이 구독 스레드라 끝나지 않는다)."""
        for _ in range(400):
            if done():
                return
            self.source.set(self.source.now() + 0.05)
            self.kros.context.timer_manager.join()
            time.sleep(0.005)
        self.fail("시간 안에 끝나지 않았다")

    def test_drives_along_and_stops_when_canceled(self) -> None:
        handle = self.client.send_goal(FollowPathGoal(self.path()), timeout=5)
        self.run_until(lambda: bool(self.commands) and self.commands[-1].linear.x > 0)
        handle.cancel(5)
        self.run_until(lambda: handle.status != GoalStatus.ACCEPTED)
        self.assertEqual(handle.result(5).result, FollowPathResult(FollowPathResult.NONE, "취소"))
        self.kros.context.topic_executor.join()
        self.assertEqual(self.commands[-1], Twist())  # 멈춤

    def test_a_new_path_replaces_the_running_one(self) -> None:
        first = self.client.send_goal(FollowPathGoal(self.path()), timeout=5)
        self.run_until(lambda: bool(self.commands))
        second = self.client.send_goal(FollowPathGoal(self.path()), timeout=5)  # bt_navigator가 다시 계획한 경로처럼
        self.run_until(lambda: first.status != GoalStatus.ACCEPTED)
        self.assertEqual(first.result(5).result, FollowPathResult(FollowPathResult.NONE, "새 경로로 바뀌었다"))
        self.assertEqual(second.status, GoalStatus.ACCEPTED)  # 이제 새 경로를 따른다
        second.cancel(5)
        self.run_until(lambda: second.status != GoalStatus.ACCEPTED)

    def test_bad_requests_have_error_codes(self) -> None:
        with self.assertLogs("kros.node.controller_server", level="WARNING"):
            for goal, code in (
                (FollowPathGoal(Path(Header(0.0, "map"))), FollowPathResult.INVALID_PATH),
                (FollowPathGoal(self.path("odom")), FollowPathResult.TF_ERROR),  # odom 좌표의 경로를 map으로 바꿀 TF가 없다
                (FollowPathGoal(self.path(), controller_id="Nope"), FollowPathResult.INVALID_CONTROLLER),
            ):
                result = self.client.send_goal(goal, timeout=5).result(5).result
                assert isinstance(result, FollowPathResult)
                self.assertEqual(result.error_code, code)


class TestNavigationInSimulation(unittest.TestCase):
    """노이즈 TurtleBot3 + EKF + Cartographer(SLAM) + planner + controller: 왼쪽 위 목표까지 실제로 간다."""

    def go(self, plugin: str, planner: str = "nav2_navfn_planner::NavfnPlanner", extra: dict[str, ParameterValue] | None = None) -> tuple[float, FollowPathResult]:
        world, robot = MazeWorld(puck_count=0), TurtleBot3(noisy=True, seed=1)
        world.add(robot)
        source = SimulatedTimeSource()
        made: dict[str, Node] = {}
        footprint = {"robot_radius": robot.radius}

        def create(ctx: Context) -> list[Node]:
            made["user"] = Node(ctx, "User")
            ekf = Ekf(ctx, parameters={"odom0": "odom", "odom0_config": ODOM_CONFIG, "imu0": "imu", "imu0_config": IMU_CONFIG})
            controller = ControllerServer(ctx, parameters={"odom_topic": "odometry/filtered", "FollowPath.plugin": plugin, **(extra or {})}, costmap_parameters=footprint)
            self.server = controller
            description = RobotStatePublisher(ctx, parameters={"robot_description": TurtleBot3.urdf_path.read_text()})
            return [made["user"], RobotBridge(ctx, robot), description, DiffDriveOdometry(ctx, parameters={"publish_tf": False}), ekf, Cartographer(ctx), PlannerServer(ctx, parameters={"GridBased.plugin": planner}, costmap_parameters=footprint), controller]

        kros = Kros(create, time_source=source)
        self.addCleanup(kros.shutdown)

        def run(seconds: float) -> None:
            for _ in range(round(seconds / 0.02)):
                world.step(0.02)
                source.set(world.time)
                kros.context.timer_manager.join()

        run(3.0)  # IMU 보정, 첫 지도
        yaw = math.pi / 2
        goal = PoseStamped(Header(0.0, "map"), Pose(Point(0.0, 1.2, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))))
        user = made["user"]
        plan = user.create_action_client("compute_path_to_pose", ComputePathToPose).send_goal(ComputePathToPoseGoal(goal), timeout=5).result(10).result
        assert isinstance(plan, ComputePathToPoseResult)
        handle = user.create_action_client("follow_path", FollowPath).send_goal(FollowPathGoal(plan.path), timeout=5)
        while handle.status == GoalStatus.ACCEPTED and world.time < 40:
            run(0.5)
        result = handle.result(5).result
        assert isinstance(result, FollowPathResult)
        return math.hypot(robot.pose.x, robot.pose.y - 1.2), result

    def test_regulated_pure_pursuit_reaches_the_goal(self) -> None:
        error, result = self.go("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
        self.assertEqual(result.error_code, FollowPathResult.NONE)
        self.assertLess(error, 0.3)  # xy_goal_tolerance 0.25 + SLAM 오차

    def test_mppi_with_smac_hybrid_reaches_the_goal(self) -> None:
        error, result = self.go("nav2_mppi_controller::MPPIController", "nav2_smac_planner::SmacPlannerHybrid")
        self.assertEqual(result.error_code, FollowPathResult.NONE)
        self.assertLess(error, 0.3)

    def test_dwb_reaches_the_goal(self) -> None:
        error, result = self.go("dwb_core::DWBLocalPlanner")
        self.assertEqual(result.error_code, FollowPathResult.NONE)
        self.assertLess(error, 0.3)


if __name__ == "__main__":
    unittest.main()
