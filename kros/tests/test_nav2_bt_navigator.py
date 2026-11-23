import math
import unittest

from kros import Kros
from kros.action import GoalStatus
from kros.context import Context
from kros.examples.turtlebot3.main import IMU_CONFIG, ODOM_CONFIG
from kros.examples.turtlebot3.robot_bridge import RobotBridge
from kros.msgs.geometry_msgs import Point, Pose, PoseStamped, Quaternion
from kros.msgs.nav2_msgs import ComputePathToPoseResult, NavigateToPose, NavigateToPoseFeedback, NavigateToPoseGoal, NavigateToPoseResult
from kros.msgs.nav_msgs import Path
from kros.msgs.std_msgs import Header
from kros.nodes import Cartographer, DiffDriveOdometry, Ekf, Node, RobotStatePublisher
from kros.nodes.nav2.behavior_server import BehaviorServer
from kros.nodes.nav2.bt_navigator import BtNavigator
from kros.nodes.nav2.controller import ControllerServer
from kros.nodes.nav2.planner import PlannerServer
from kros.timer import SimulatedTimeSource
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3


def pose(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(Header(0.0, "map"), Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))))


class Navigation:
    """노이즈 TurtleBot3 + EKF + Cartographer + planner(Smac Hybrid) + controller(MPPI) + bt_navigator: 예제 main.py와 같은 조립."""

    def __init__(self, test: unittest.TestCase) -> None:
        self.world, self.robot = MazeWorld(puck_count=0), TurtleBot3(noisy=True, seed=1)
        self.world.add(self.robot)
        self.source = SimulatedTimeSource()
        made: dict[str, Node] = {}
        footprint = {"robot_radius": self.robot.radius}

        def create(ctx: Context) -> list[Node]:
            made["user"] = Node(ctx, "User")
            ekf = Ekf(ctx, parameters={"odom0": "odom", "odom0_config": ODOM_CONFIG, "imu0": "imu", "imu0_config": IMU_CONFIG})
            description = RobotStatePublisher(ctx, parameters={"robot_description": TurtleBot3.urdf_path.read_text()})
            planner = PlannerServer(ctx, parameters={"GridBased.plugin": "nav2_smac_planner::SmacPlannerHybrid"}, costmap_parameters=footprint)
            controller = ControllerServer(ctx, parameters={"odom_topic": "odometry/filtered", "FollowPath.plugin": "nav2_mppi_controller::MPPIController"}, costmap_parameters=footprint)
            odometry = DiffDriveOdometry(ctx, parameters={"publish_tf": False})
            return [made["user"], RobotBridge(ctx, self.robot), description, odometry, ekf, Cartographer(ctx), planner, controller, BehaviorServer(ctx), BtNavigator(ctx)]

        self.kros = Kros(create, time_source=self.source)
        test.addCleanup(self.kros.shutdown)
        self.user = made["user"]
        self.local_plans: list[Path] = []
        self.user.subscribe("local_plan", Path, self.local_plans.append)
        self.run(3.0)  # IMU 보정, 첫 지도

    def run(self, seconds: float) -> None:
        for _ in range(round(seconds / 0.02)):
            self.world.step(0.02)
            self.source.set(self.world.time)
            self.kros.context.timer_manager.join()

    def error_to(self, x: float, y: float) -> float:
        return math.hypot(self.robot.pose.x - x, self.robot.pose.y - y)


class TestBtNavigator(unittest.TestCase):
    def test_navigate_to_pose_plans_follows_and_reports_progress(self) -> None:
        nav = Navigation(self)
        feedback: list[NavigateToPoseFeedback] = []
        handle = nav.user.create_action_client("navigate_to_pose", NavigateToPose).send_goal(NavigateToPoseGoal(pose(0.0, 1.2, math.pi / 2)), feedback.append, timeout=5)
        while handle.status == GoalStatus.ACCEPTED and nav.world.time < 40:
            nav.run(0.5)
        result = handle.result(5).result
        assert isinstance(result, NavigateToPoseResult)
        self.assertEqual(result.error_code, NavigateToPoseResult.NONE, result.error_msg)
        self.assertLess(nav.error_to(0.0, 1.2), 0.3)
        self.assertTrue(feedback)
        self.assertGreater(max(f.distance_remaining for f in feedback), 1.0)  # 처음엔 멀었다
        self.assertTrue(nav.local_plans and nav.local_plans[0].header.frame_id == "odom")  # controller가 예상 자취를 냈다 (local costmap 좌표, nav2와 같다)

    def test_goal_pose_topic_starts_navigation_like_rviz(self) -> None:
        nav = Navigation(self)
        nav.user.publish("goal_pose", pose(0.6, 0.9, 0.0))  # rviz의 2D Goal Pose
        for _ in range(60):
            nav.run(0.5)
            if nav.error_to(0.6, 0.9) < 0.3 and abs(nav.robot.pose.theta) < 0.3:
                break
        self.assertLess(nav.error_to(0.6, 0.9), 0.3)  # 도착 허용 0.25m(map 기준) + SLAM 위치 오차

    def test_unreachable_goal_fails_with_the_planner_error(self) -> None:
        nav = Navigation(self)
        with self.assertLogs("kros.node.bt_navigator", level="WARNING"):
            handle = nav.user.create_action_client("navigate_to_pose", NavigateToPose).send_goal(NavigateToPoseGoal(pose(30.0, 0.0, 0.0)), timeout=5)
            result = handle.result(10).result
        assert isinstance(result, NavigateToPoseResult)
        self.assertEqual(result.error_code, ComputePathToPoseResult.GOAL_OUTSIDE_MAP)


if __name__ == "__main__":
    unittest.main()
