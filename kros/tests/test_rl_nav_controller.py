import importlib.util
import math
import unittest

from kros import Kros
from kros.msgs.sensor_msgs import LaserScan
from kros.msgs.std_msgs import Header
from kros.nodes import Node
from kros.nodes.nav2.controller.controller_server import PLUGINS
from kros.nodes.nav2.costmap_2d import Costmap
from kros.examples.turtlebot3.rl_nav_controller import RlNavController
from kros.parameter import ParameterValue
from kros.timer import SimulatedTimeSource

HAS_ORT = importlib.util.find_spec("onnxruntime") is not None
try:
    from kros.examples.turtlebot3.main import RL_POLICY
except ImportError:  # 예제를 못 읽는 프로필
    RL_POLICY = None  # type: ignore[assignment]


def make(test: unittest.TestCase, **parameters: ParameterValue) -> RlNavController:
    made: dict[str, Node] = {}
    kros = Kros(lambda ctx: [made.setdefault("n", Node(ctx, "controller_server", parameters={f"FollowPath.{k}": v for k, v in parameters.items()}))], time_source=SimulatedTimeSource())
    test.addCleanup(kros.shutdown)
    controller = RlNavController()
    controller.configure(made["n"], "FollowPath")
    return controller


def scan(ranges: list[float], frame: str = "base_scan") -> LaserScan:
    inc = 2 * math.pi / len(ranges)
    return LaserScan(Header(0.0, frame), 0.0, 2 * math.pi - inc, inc, 0.12, 3.5, tuple(ranges))


class TestObservation(unittest.TestCase):
    """관측 = 실기 rl_nav_controller.cpp의 waypoints()/scan_rays()와 같은 식."""

    def test_registered_as_a_controller_plugin(self) -> None:
        self.assertIs(PLUGINS["rl_nav_controller::RlNavController"], RlNavController)

    def test_waypoints_every_wp_step_in_robot_frame_then_the_last_point(self) -> None:
        rl = make(self)
        path = [(x * 0.05, 0.0, 0.0) for x in range(21)]  # 1m 직선 (촘촘한 planner 경로)
        self.assertEqual([round(v, 6) for v in rl.waypoints((0.0, 0.0, 0.0), path)], [0.15, 0.0, 0.3, 0.0, 0.45, 0.0, 0.5, 0.0, 0.5, 0.0])  # 0.3m 간격 / 2.0, 끝 1m에서 멈춤
        turned = rl.waypoints((0.0, 0.0, math.pi / 2), path)  # 로봇이 왼쪽(+y)을 본다: 경로는 오른쪽
        self.assertAlmostEqual(turned[0], 0.0, places=6)
        self.assertAlmostEqual(turned[1], -0.15, places=6)
        self.assertEqual([round(v, 6) for v in rl.waypoints((0.5, 0.0, 0.0), path)[:2]], [0.15, 0.0])  # 지나온 점은 버리고 가장 가까운 점부터

    def test_scan_rays_nearest_beam_minus_inflation(self) -> None:
        rl = make(self)
        ranges = [2.0] * 360
        ranges[90] = 1.0  # 왼쪽 90도에 가까운 것
        ranges[180] = math.inf  # 뒤쪽은 측정 없음
        rays = rl.scan_rays(scan(ranges), 0.0, 0.2)
        self.assertEqual(len(rays), 36)
        self.assertAlmostEqual(rays[9], (1.0 - 0.2) / 3.5)  # 36방향의 9번 = 90도
        self.assertAlmostEqual(rays[0], (2.0 - 0.2) / 3.5)
        self.assertAlmostEqual(rays[18], (2.0 - 0.2) / 3.5)  # 180도 빔은 없어도 +-1 빔이 있다
        ranges[179:182] = [math.inf] * 3
        self.assertAlmostEqual(rl.scan_rays(scan(ranges), 0.0, 0.2)[18], (3.5 - 0.2) / 3.5)  # 셋 다 없으면 lidar_max
        shifted = rl.scan_rays(scan(ranges), math.pi / 2, 0.2)  # 라이다가 90도 돌아 달렸다
        self.assertAlmostEqual(shifted[18], (1.0 - 0.2) / 3.5)  # 로봇 기준 180도 = 라이다 기준 90도

    def test_without_a_model_or_disabled_it_drives_with_the_fallback(self) -> None:
        costmap = Costmap(0.05, 60, 60, -1.5, -1.5)
        path = [(x * 0.05, 0.0, 0.0) for x in range(21)]
        for parameters in ({"enable": True}, {"enable": False, "model_path": "nothing.onnx"}):
            rl = make(self, **parameters)
            rl.scan = scan([2.0] * 360)
            v, _ = rl.compute_velocity_commands((0.0, 0.0, 0.0), (0.0, 0.0), path, costmap, 0.05)
            self.assertFalse(rl.active)  # 정책이 아니라 MPPI가 냈다
            self.assertGreaterEqual(v, 0.0)


@unittest.skipUnless(HAS_ORT and RL_POLICY is not None and RL_POLICY.exists(), "onnxruntime 또는 isaac의 학습 정책이 없다")
class TestTrainedPolicy(unittest.TestCase):
    def test_policy_drives_and_reaches_the_goal_in_simulation(self) -> None:
        from kros.tests.test_nav2_controller import TestNavigationInSimulation

        sim = TestNavigationInSimulation("test_regulated_pure_pursuit_reaches_the_goal")
        sim.addCleanup = self.addCleanup  # type: ignore[method-assign]
        parameters: dict[str, ParameterValue] = {"FollowPath.enable": True, "FollowPath.model_path": str(RL_POLICY), "FollowPath.lidar_inflation": 0.03}
        error, result = sim.go("rl_nav_controller::RlNavController", extra=parameters)
        self.assertEqual(result.error_code, 0, result.error_msg)
        self.assertLess(error, 0.35)
        rl = sim.server.controllers["FollowPath"]
        assert isinstance(rl, RlNavController)
        self.assertGreater(rl.policy_steps, 50)  # MPPI(fallback)가 아니라 정책이 몰았다


if __name__ == "__main__":
    unittest.main()
