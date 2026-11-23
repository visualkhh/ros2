import math
import time
import unittest

from kros import Kros
from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from kros.msgs.sensor_msgs import LaserScan
from kros.msgs.nav_msgs import FREE, OCCUPIED, UNKNOWN, MapMetaData, OccupancyGrid
from kros.msgs.std_msgs import Header
from kros.nodes import Node
from kros.nodes.nav2.costmap_2d import FREE_SPACE, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION, Costmap, Costmap2DROS
from kros.nodes.nav2.costmap_2d.costmap import translate
from kros.parameter import ParameterValue
from kros.tf import TransformBroadcaster
from kros.timer import SimulatedTimeSource
from kros.topic import QOS_LATCHED

RES = 0.05


def wall_grid(width: int = 41, height: int = 41, unknown_column: int | None = None) -> OccupancyGrid:
    """가운데 칸 하나만 벽인 지도 (origin (-1, -1)). unknown_column 줄은 모름."""
    data = [FREE] * (width * height)
    data[(height // 2) * width + width // 2] = OCCUPIED
    if unknown_column is not None:
        for j in range(height):
            data[j * width + unknown_column] = UNKNOWN
    return OccupancyGrid(Header(0.0, "map"), MapMetaData(0.0, RES, width, height, Pose(Point(-1.0, -1.0, 0.0))), tuple(data))


class TestCostmap(unittest.TestCase):
    def test_static_layer_reads_walls_free_and_unknown(self) -> None:
        costmap = Costmap.from_occupancy_grid(wall_grid(unknown_column=0))
        self.assertEqual(costmap.cost(20, 20), LETHAL_OBSTACLE)
        self.assertEqual(costmap.cost(10, 10), FREE_SPACE)
        self.assertEqual(costmap.cost(0, 5), NO_INFORMATION)
        self.assertEqual(Costmap.from_occupancy_grid(wall_grid(unknown_column=0), track_unknown_space=False).cost(0, 5), FREE_SPACE)
        self.assertEqual(costmap.cost_at(0.0, 0.0), LETHAL_OBSTACLE)  # 칸 (20, 20) = (-1 + 20 * 0.05 ~ ) 원점
        self.assertEqual(costmap.cost_at(5.0, 0.0), NO_INFORMATION)  # 지도 밖

    def test_inflation_follows_the_nav2_formula(self) -> None:
        costmap = Costmap.from_occupancy_grid(wall_grid())
        costmap.inflate(inscribed_radius=0.2, inflation_radius=0.55, cost_scaling_factor=3.0)
        self.assertEqual(costmap.cost(20, 20), LETHAL_OBSTACLE)
        self.assertEqual(costmap.cost(24, 20), INSCRIBED_INFLATED_OBSTACLE)  # 0.2m: 로봇 몸이 닿는다
        self.assertEqual(costmap.cost(26, 20), int(252 * math.exp(-3.0 * (0.3 - 0.2))))  # 0.3m
        self.assertEqual(costmap.cost(31, 20), int(252 * math.exp(-3.0 * (0.55 - 0.2))))  # inflation_radius 끝
        self.assertEqual(costmap.cost(32, 20), FREE_SPACE)  # 그 밖
        self.assertGreater(costmap.cost(23, 20), costmap.cost(26, 20))  # 가까울수록 비싸다 (inscribed 253 > 0.3m)

    def test_unknown_is_covered_only_where_the_body_would_touch(self) -> None:
        costmap = Costmap.from_occupancy_grid(wall_grid(unknown_column=24))  # 벽에서 0.2m 떨어진 줄이 모름
        costmap.inflate(0.15, 0.55, 3.0)
        self.assertEqual(costmap.cost(24, 20), NO_INFORMATION)  # 0.2m > 0.15m: 모름 그대로 (nav2 inflate_unknown false)
        costmap = Costmap.from_occupancy_grid(wall_grid(unknown_column=22))  # 0.1m
        costmap.inflate(0.15, 0.55, 3.0)
        self.assertEqual(costmap.cost(22, 20), INSCRIBED_INFLATED_OBSTACLE)

    def test_cost_translation_table_like_nav2(self) -> None:
        self.assertEqual([translate(c) for c in (0, 1, 252, 253, 254, 255)], [0, 1, 98, 99, 100, UNKNOWN])

    def test_inflating_a_maze_sized_map_is_quick_enough(self) -> None:
        width, height = 100, 130  # 미로 5 x 6.5 m (0.05m 칸)
        data = [OCCUPIED if i in (0, width - 1) or j in (0, height - 1) or (i % 20 == 0 and j < 100) else FREE for j in range(height) for i in range(width)]
        costmap = Costmap.from_occupancy_grid(OccupancyGrid(Header(0.0), MapMetaData(0.0, RES, width, height), tuple(data)))
        start = time.perf_counter()
        costmap.inflate(0.2, 0.55, 3.0)
        self.assertLess(time.perf_counter() - start, 2.0)


class TestCostmap2DROS(unittest.TestCase):
    def setUp(self) -> None:
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["user"], made["costmap"] = Node(ctx, "User"), Costmap2DROS(ctx, parameters={"robot_radius": 0.2})
            return list(made.values())

        self.kros = Kros(create, time_source=SimulatedTimeSource())
        self.addCleanup(self.kros.shutdown)
        costmap = made["costmap"]
        assert isinstance(costmap, Costmap2DROS)
        self.node, self.user = costmap, made["user"]
        self.published: list[OccupancyGrid] = []
        self.user.subscribe("global_costmap/costmap", OccupancyGrid, self.published.append, QOS_LATCHED)

    def test_builds_from_the_map_publishes_and_rebuilds_on_parameter_change(self) -> None:
        self.assertIsNone(self.node.costmap)  # 지도 전에는 없다
        self.user.publish("map", wall_grid(), QOS_LATCHED)
        self.kros.context.topic_executor.join()
        costmap = self.node.costmap
        assert costmap is not None
        self.assertEqual(costmap.cost(24, 20), INSCRIBED_INFLATED_OBSTACLE)
        self.assertEqual(len(self.published), 1)
        self.assertEqual(self.published[0].data[20 * 41 + 20], 100)  # 벽 = 100
        self.node.set_parameter("robot_radius", 0.1)  # 더 작은 로봇: 0.2m 칸은 이제 몸이 안 닿는다
        self.kros.context.topic_executor.join()
        rebuilt = self.node.costmap
        assert rebuilt is not None
        self.assertLess(rebuilt.cost(24, 20), INSCRIBED_INFLATED_OBSTACLE)
        self.assertIsNot(rebuilt, costmap)  # 받은 것은 그대로 두고 새로 만든다

    def test_an_empty_map_means_no_map(self) -> None:
        self.user.publish("map", wall_grid(), QOS_LATCHED)
        self.kros.context.topic_executor.join()
        self.assertIsNotNone(self.node.costmap)
        self.user.publish("map", OccupancyGrid(Header(0.0, "map")), QOS_LATCHED)  # map_server가 지도를 내렸다
        self.kros.context.topic_executor.join()
        self.assertIsNone(self.node.costmap)
        self.assertEqual(self.published[-1].info.width, 0)

    def test_robot_pose_comes_from_tf(self) -> None:
        self.assertIsNone(self.node.get_robot_pose())
        yaw = 0.5
        transform = Transform(Vector3(1.0, 2.0, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        TransformBroadcaster(self.user).send_transform(TransformStamped(Header(0.0, "map"), "base_footprint", transform))
        self.kros.context.topic_executor.join()
        pose = self.node.get_robot_pose()
        assert pose is not None
        self.assertAlmostEqual(pose[0], 1.0)
        self.assertAlmostEqual(pose[1], 2.0)
        self.assertAlmostEqual(pose[2], yaw)



class TestRollingObstacleCostmap(unittest.TestCase):
    """nav2 local costmap처럼: 지도 없이 로봇을 따라다니는 창 + 라이다 장애물."""

    def setUp(self) -> None:
        made: dict[str, Node] = {}
        parameters: dict[str, ParameterValue] = {"global_frame": "odom", "plugins": ("obstacle_layer", "inflation_layer"), "rolling_window": True, "width": 2.0, "height": 2.0, "robot_radius": 0.1, "inflation_radius": 0.3}

        def create(ctx: Context) -> list[Node]:
            made["user"], made["costmap"] = Node(ctx, "User"), Costmap2DROS(ctx, "local_costmap", parameters)
            return list(made.values())

        self.source = SimulatedTimeSource()
        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        costmap = made["costmap"]
        assert isinstance(costmap, Costmap2DROS)
        self.node, self.user = costmap, made["user"]
        self.tf = TransformBroadcaster(self.user)
        self.robot_at(0.0, 0.0)

    def robot_at(self, x: float, t: float) -> None:
        self.tf.send_transform(TransformStamped(Header(t, "odom"), "base_footprint", Transform(Vector3(x, 0.0, 0.0), Quaternion())))
        self.tf.send_transform(TransformStamped(Header(t, "base_footprint"), "base_scan", Transform()))
        self.kros.context.topic_executor.join()

    def scan(self, front: float, t: float) -> None:
        """앞쪽(0도 둘레 10도)만 front 거리에 무언가 (inf면 없음), 나머지 방향은 측정 없음."""
        inc = math.radians(1)
        ranges = tuple(front if (i <= 5 or i >= 355) else math.inf for i in range(360))
        self.user.publish("scan", LaserScan(Header(t, "base_scan"), 0.0, 2 * math.pi - inc, inc, 0.12, 3.5, ranges))
        self.kros.context.topic_executor.join()

    def tick(self, t: float) -> None:
        self.source.set(t)
        self.kros.context.timer_manager.join()
        self.kros.context.topic_executor.join()

    def test_marks_obstacles_without_a_map_clears_them_and_follows_the_robot(self) -> None:
        self.scan(0.63, 0.1)  # 칸 한가운데 (칸 경계 0.6에 두면 부동소수점으로 옆 칸에 들어간다)
        self.tick(0.3)
        costmap = self.node.costmap
        assert costmap is not None  # 지도가 없어도 생긴다
        self.assertEqual((costmap.width, costmap.height), (40, 40))
        self.assertEqual(costmap.cost_at(0.63, 0.0), LETHAL_OBSTACLE)  # 라이다가 본 것
        self.assertEqual(costmap.cost_at(0.0, -0.8), FREE_SPACE)
        self.assertGreater(costmap.cost_at(0.45, 0.0), 0)  # 둘레 inflation
        self.scan(math.inf, 0.4)  # 치웠다: 레이가 지나가며 비운다
        self.tick(0.6)
        cleared = self.node.costmap
        assert cleared is not None
        self.assertEqual(cleared.cost_at(0.63, 0.0), FREE_SPACE)
        self.robot_at(1.0, 0.7)
        self.tick(0.9)
        moved = self.node.costmap
        assert moved is not None
        self.assertAlmostEqual(moved.origin_x, 0.0, places=6)  # 창이 로봇(1.0)을 가운데 두고 따라왔다
        published: list[OccupancyGrid] = []
        self.user.subscribe("local_costmap/costmap", OccupancyGrid, published.append, QOS_LATCHED)
        self.kros.context.topic_executor.join()
        self.assertEqual(published[-1].header.frame_id, "odom")


if __name__ == "__main__":
    unittest.main()
