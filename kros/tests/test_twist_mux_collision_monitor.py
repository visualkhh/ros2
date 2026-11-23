import math
import unittest

from kros import Kros
from kros.context import Context
from kros.msgs.geometry_msgs import Transform, TransformStamped, Twist, Vector3
from kros.msgs.sensor_msgs import LaserScan
from kros.msgs.std_msgs import Bool, Header
from kros.nodes import Node
from kros.nodes.nav2.collision_monitor import CollisionMonitor
from kros.nodes.twist_mux import TwistMux
from kros.parameter import ParameterValue
from kros.tf import StaticTransformBroadcaster
from kros.timer import SimulatedTimeSource


def twist(v: float, w: float = 0.0) -> Twist:
    return Twist(Vector3(x=v), Vector3(z=w))


class Setup(unittest.TestCase):
    def make(self, node_class: type[Node], parameters: dict[str, ParameterValue], out: str) -> None:
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["user"], made["node"] = Node(ctx, "User"), node_class(ctx, parameters=parameters)
            return list(made.values())

        self.source = SimulatedTimeSource()
        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        self.user, self.node = made["user"], made["node"]
        self.out: list[Twist] = []
        self.user.subscribe(out, Twist, self.out.append)

    def send(self, topic: str, message: object, at: float | None = None) -> None:
        if at is not None:
            self.source.set(at)
        self.user.publish(topic, message)
        self.kros.context.topic_executor.join()


class TestTwistMux(Setup):
    def setUp(self) -> None:
        self.make(TwistMux, {"locks": ("e_stop",), "e_stop.priority": 200}, "cmd_vel_out")

    def test_highest_priority_alive_input_wins(self) -> None:
        self.send("cmd_vel_nav", twist(0.2), at=1.0)
        self.assertEqual(self.out[-1], twist(0.2))  # 내비게이션만 있다
        self.send("cmd_vel_teleop", twist(0.0, 1.0), at=1.1)
        self.assertEqual(self.out[-1], twist(0.0, 1.0))  # 사람(teleop, 100)이 내비게이션(10)을 이긴다
        count = len(self.out)
        self.send("cmd_vel_nav", twist(0.2), at=1.2)
        self.assertEqual(len(self.out), count)  # teleop이 살아 있는 동안 내비게이션은 막힌다
        self.send("cmd_vel_nav", twist(0.2), at=2.0)  # teleop이 0.5초 넘게 조용하다
        self.assertEqual(self.out[-1], twist(0.2))

    def test_lock_blocks_lower_priorities(self) -> None:
        self.send("e_stop", Bool(True), at=1.0)  # 비상정지 (200)
        self.send("cmd_vel_teleop", twist(0.1), at=1.1)
        self.send("cmd_vel_nav", twist(0.2), at=1.2)
        self.assertEqual(self.out, [])
        self.send("e_stop", Bool(False), at=1.3)
        self.send("cmd_vel_teleop", twist(0.1), at=1.4)
        self.assertEqual(self.out, [twist(0.1)])


def scan_with_obstacle_at(distance: float, width: float = 0.2) -> LaserScan:
    """앞쪽 width 폭의 벽이 distance에 있는 스캔 (나머지는 측정 없음)."""
    inc = math.radians(1)
    ranges = []
    for i in range(360):
        a = i * inc if i < 180 else i * inc - 2 * math.pi
        y = distance * math.tan(a)
        ranges.append(distance / math.cos(a) if abs(a) < math.pi / 2 and abs(y) <= width / 2 else math.inf)
    return LaserScan(Header(0.0, "base_scan"), 0.0, 2 * math.pi - inc, inc, 0.12, 3.5, tuple(ranges))


class TestCollisionMonitor(Setup):
    def setUp(self) -> None:
        self.make(CollisionMonitor, {"cmd_vel_in_topic": "cmd_vel_mux", "polygons": ("PolygonStop", "PolygonSlow", "Approach"), "Approach.action_type": "approach", "Approach.type": "circle", "Approach.radius": 0.25, "Approach.enabled": False}, "cmd_vel")
        StaticTransformBroadcaster(self.user).send_transform(TransformStamped(Header(0.0, "base_footprint"), "base_scan", Transform(Vector3(-0.05, 0.0, 0.0))))
        self.kros.context.topic_executor.join()

    def obstacle(self, distance: float) -> None:
        """base_footprint에서 distance 앞 (라이다는 0.05m 뒤에 있다)."""
        self.send("scan", scan_with_obstacle_at(distance + 0.05))

    def test_without_scan_it_stops(self) -> None:
        self.send("cmd_vel_mux", twist(0.2))
        self.assertEqual(self.out[-1], twist(0.0))
        self.assertIn("no fresh observation", self.node.state)  # type: ignore[attr-defined]

    def test_stop_and_slowdown_polygons(self) -> None:
        self.obstacle(2.0)
        self.send("cmd_vel_mux", twist(0.2, 0.5))
        self.assertEqual(self.out[-1], twist(0.2, 0.5))  # 멀다: 그대로
        self.obstacle(0.4)
        self.send("cmd_vel_mux", twist(0.2, 0.5))
        self.assertAlmostEqual(self.out[-1].linear.x, 0.2 * 0.3)  # 0.5m 상자: 감속
        self.assertEqual(self.node.state, "PolygonSlow: slowdown")  # type: ignore[attr-defined]
        self.obstacle(0.2)
        self.send("cmd_vel_mux", twist(0.2, 0.5))
        self.assertEqual(self.out[-1], twist(0.0))  # 0.3m 상자: 정지 (둘 다 걸리면 더 느린 것)

    def test_approach_slows_in_proportion_to_time_before_collision(self) -> None:
        for name in ("PolygonStop", "PolygonSlow"):
            self.node.set_parameter(f"{name}.enabled", False)
        self.node.set_parameter("Approach.enabled", True)
        self.obstacle(0.75)  # 반지름 0.25 원이 닿으려면 0.5m: 0.2m/s면 2.5초 뒤 (time_before_collision 2초보다 늦다)
        self.send("cmd_vel_mux", twist(0.2))
        self.assertEqual(self.out[-1], twist(0.2))
        self.send("cmd_vel_mux", twist(0.5))  # 1초 뒤 닿는다: 2초의 절반 -> 반으로
        self.assertAlmostEqual(self.out[-1].linear.x, 0.5 * 1.0 / 2.0, delta=0.05)

    def test_stale_scan_stops(self) -> None:
        self.obstacle(2.0)
        self.send("cmd_vel_mux", twist(0.2), at=3.0)  # 스캔이 2초(source_timeout)보다 오래됐다
        self.assertEqual(self.out[-1], twist(0.0))


if __name__ == "__main__":
    unittest.main()
