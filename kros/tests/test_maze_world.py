import math
import pathlib
import re
import unittest

from kros.msgs.geometry_msgs import Pose2D
from kros.worlds.clock import Clock
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.maze.track import WALL_THICKNESS, WALLS
from kros.worlds.objects import Puck, PuckShape
from kros.worlds.world import World
from kros.worlds.world_object import WorldObject


class DummyObject(WorldObject):
    def __init__(self) -> None:
        self.clocks: list[Clock] = []

    def tick(self, clock: Clock, world: World) -> None:
        self.clocks.append(clock)


class TestMazeWorld(unittest.TestCase):
    world = MazeWorld(puck_count=0)  # 벽만 있는 월드

    def test_axis_directions_from_start(self) -> None:
        r = self.world.ray_distance
        self.assertAlmostEqual(r(0.0, 0.0, 0.0), 1.145)  # 동쪽 안쪽 벽 x=1.17의 앞면
        self.assertAlmostEqual(r(0.0, 0.0, math.pi), 2.215)  # 서쪽 바깥벽
        self.assertAlmostEqual(r(0.0, 0.0, math.pi / 2), 1.785)  # 북쪽 안쪽 벽 y=1.81
        self.assertAlmostEqual(r(0.0, 0.0, -math.pi / 2), 3.175)  # 남쪽: x=0은 안쪽 벽이 없어 바깥벽

    def test_inner_wall_occludes_outer(self) -> None:
        r = self.world.ray_distance
        self.assertAlmostEqual(r(0.8, 0.0, -math.pi / 2), 1.135)  # x=0.8은 안쪽 벽(0.41..1.19) 위
        self.assertAlmostEqual(r(0.8, -1.5, -math.pi / 2), 0.415)  # 그 벽 아래에서는 y=-1.94

    def test_ray_missing_everything_is_inf(self) -> None:
        self.assertEqual(self.world.ray_distance(10.0, 10.0, 0.0), math.inf)

    def test_every_hit_lies_on_a_wall_box(self) -> None:
        # 예전 계산은 168도에서 벽 끝 옆 공중(-1.058, 1.065)을 맞았다고 했다
        h = WALL_THICKNESS / 2
        for deg in range(360):
            a = math.radians(deg)
            r = self.world.ray_distance(1.6, 0.5, a)
            if r == math.inf:
                continue
            px, py = 1.6 + r * math.cos(a), 0.5 + r * math.sin(a)
            self.assertTrue(
                any(min(p, q) - h - 1e-6 <= px <= max(p, q) + h + 1e-6 and min(u, v) - h - 1e-6 <= py <= max(u, v) + h + 1e-6 for p, u, q, v in WALLS),
                (deg, px, py),
            )

    def test_push_out_of_wall(self) -> None:
        x, y = self.world.push_out(2.1, 0.0, 0.2)  # 동쪽 바깥벽(x=2.24)을 파고듦
        self.assertAlmostEqual(x, 2.24 - WALL_THICKNESS / 2 - 0.2)
        self.assertEqual(self.world.push_out(0.0, 0.0, 0.2), (0.0, 0.0))  # 빈 곳은 그대로

    def test_start_pose_is_free(self) -> None:
        p = self.world.start_pose
        self.assertEqual(self.world.push_out(p.x, p.y, 0.2), (p.x, p.y))

    def test_step_gives_every_object_the_clock(self) -> None:
        world = MazeWorld(puck_count=0)  # 벽만 있는 월드
        a, b = DummyObject(), DummyObject()
        world.add(a)
        world.add(b)
        world.step(0.1)
        world.step(0.25)
        self.assertEqual(a.clocks, [Clock(0.1, 0.1), Clock(0.35, 0.25)])
        self.assertEqual(b.clocks, a.clocks)

    def test_add_places_object_at_start_pose_unless_given(self) -> None:
        world, a, b = MazeWorld(puck_count=0), DummyObject(), DummyObject()
        world.add(a)
        world.add(b, Pose2D(1.0, 2.0, 0.5))
        self.assertEqual(a.pose, world.start_pose)
        self.assertEqual(b.pose, Pose2D(1.0, 2.0, 0.5))

    def test_ten_pucks_scattered_in_free_space(self) -> None:
        world = MazeWorld(seed=1)
        pucks = [o for o in world.objects if isinstance(o, Puck)]
        self.assertEqual(len(pucks), 10)
        self.assertEqual({p.shape for p in pucks}, set(PuckShape))  # 모든 모양이 나온다
        self.assertEqual(len({p.color for p in pucks}), 10)  # 색이 다 다르다
        for p in pucks:
            x, y = p.pose.x, p.pose.y
            self.assertEqual(world.push_out(x, y, p.radius + 0.10), (x, y))  # 벽에서 10cm 이상
            self.assertGreaterEqual(math.hypot(x - world.start_pose.x, y - world.start_pose.y), 0.5)
        for i, a in enumerate(pucks):
            for b in pucks[i + 1 :]:
                self.assertGreaterEqual(math.hypot(a.pose.x - b.pose.x, a.pose.y - b.pose.y), 0.3)

    def test_same_seed_same_layout_and_default_is_random(self) -> None:
        def layout(w: MazeWorld) -> list[tuple[float, float]]:
            return [(o.pose.x, o.pose.y) for o in w.objects]

        self.assertEqual(layout(MazeWorld(seed=3)), layout(MazeWorld(seed=3)))
        self.assertNotEqual(layout(MazeWorld(seed=3)), layout(MazeWorld(seed=4)))

    def test_pucks_do_not_overlap_after_step(self) -> None:
        world = MazeWorld(puck_count=0)
        a, b = Puck(PuckShape.CIRCLE, (1, 0, 0)), Puck(PuckShape.SQUARE, (0, 1, 0))
        world.add(a, Pose2D(0.0, 0.0, 0.0))
        world.add(b, Pose2D(0.05, 0.0, 0.0))  # 겹쳐 놓음
        world.step(0.1)
        self.assertGreaterEqual(math.hypot(a.pose.x - b.pose.x, a.pose.y - b.pose.y), a.radius + b.radius - 1e-6)

    def test_lighter_object_is_pushed_more(self) -> None:
        world = MazeWorld(puck_count=0)
        light, heavy = Puck(PuckShape.CIRCLE, (1, 0, 0)), Puck(PuckShape.CIRCLE, (0, 1, 0))
        heavy.mass = light.mass * 9
        world.add(light, Pose2D(0.0, 0.0, 0.0))
        world.add(heavy, Pose2D(0.1, 0.0, 0.0))
        world.step(0.1)
        self.assertGreater(abs(light.pose.x - 0.0), 8 * abs(heavy.pose.x - 0.1))

    def test_pushed_object_stays_out_of_walls(self) -> None:
        world = MazeWorld(puck_count=0)
        a, b = Puck(PuckShape.CIRCLE, (1, 0, 0)), Puck(PuckShape.CIRCLE, (0, 1, 0))
        world.add(a, Pose2D(2.17, 0.0, 0.0))  # 동쪽 바깥벽에 붙어 있음
        world.add(b, Pose2D(1.9, 0.0, 0.0))
        for _ in range(5):
            world.step(0.1)
        x, y = world.push_out(a.pose.x, a.pose.y, a.radius)
        self.assertAlmostEqual(x, a.pose.x)
        self.assertAlmostEqual(y, a.pose.y)

    def test_ray_sees_object_taller_than_the_ray_but_not_the_one_it_starts_inside(self) -> None:
        world = MazeWorld(puck_count=0)
        tall = Puck(PuckShape.CIRCLE, (1, 0, 0), height=0.3)
        world.add(tall, Pose2D(0.5, 0.0, 0.0))
        self.assertAlmostEqual(world.ray_distance(0.0, 0.0, 0.0, 0.1), 0.5 - tall.radius)
        self.assertAlmostEqual(world.ray_distance(0.5, 0.0, 0.0, 0.1), 1.145 - 0.5)  # 객체 안에서 쏘면 그 객체는 무시, 벽까지
        self.assertAlmostEqual(world.ray_distance(0.0, 0.0, math.pi / 2, 0.1), 1.785)  # 객체 옆으로 쏘면 벽

    def test_ray_passes_over_object_lower_than_the_ray(self) -> None:
        world = MazeWorld(puck_count=0)
        low = Puck(PuckShape.CIRCLE, (1, 0, 0), height=0.04)
        world.add(low, Pose2D(0.5, 0.0, 0.0))
        self.assertAlmostEqual(world.ray_distance(0.0, 0.0, 0.0, 0.132), 1.145)  # 라이다 높이면 퍽 위로 지나 벽까지
        self.assertAlmostEqual(world.ray_distance(0.0, 0.0, 0.0, 0.02), 0.5 - low.radius)  # 더 낮게 쏘면 보인다

    def test_ray_over_the_walls_sees_nothing(self) -> None:
        self.assertEqual(self.world.ray_distance(0.0, 0.0, 0.0, 0.5), math.inf)  # 벽(0.25m)보다 높게 쏘면 아무것도

    def test_weight_is_mass_times_gravity(self) -> None:
        puck = Puck(PuckShape.CIRCLE, (1, 0, 0))
        self.assertAlmostEqual(MazeWorld(puck_count=0).weight(puck), puck.mass * 9.81)
        self.assertAlmostEqual(MazeWorld(puck_count=0, gravity=1.62).weight(puck), puck.mass * 1.62)  # 같은 질량, 달에서는 가볍다

    def test_pushed_puck_slides_then_stops_by_friction(self) -> None:
        world = MazeWorld(puck_count=0)
        puck = Puck(PuckShape.CIRCLE, (1, 0, 0))
        world.add(puck, Pose2D(0.0, 0.0, 0.0))
        puck.velocity = (1.0, 0.0)  # 1 m/s로 밀렸다
        for _ in range(100):
            world.step(0.01)
        self.assertEqual(puck.velocity, (0.0, 0.0))  # 멈췄다
        self.assertAlmostEqual(puck.pose.x, 1.0**2 / (2 * world.friction_with_ground(puck) * world.gravity), places=1)  # 정지거리 v^2/(2 mu g)

    def test_more_friction_stops_sooner(self) -> None:
        world = MazeWorld(puck_count=0)
        slippery, rough = Puck(PuckShape.CIRCLE, (1, 0, 0)), Puck(PuckShape.CIRCLE, (0, 1, 0))
        slippery.friction, rough.friction = 0.1, 0.6
        world.add(slippery, Pose2D(-1.0, -2.0, 0.0))
        world.add(rough, Pose2D(-1.0, -2.5, 0.0))
        slippery.velocity = rough.velocity = (0.8, 0.0)
        for _ in range(50):
            world.step(0.01)
        self.assertGreater(slippery.pose.x + 1.0, rough.pose.x + 1.0)

    def test_world_gravity_and_ground_friction_change_the_sliding(self) -> None:
        def slide_distance(gravity: float = 9.81, ground_friction: float = 0.3) -> float:
            world = MazeWorld(puck_count=0, gravity=gravity, ground_friction=ground_friction)
            puck = Puck(PuckShape.CIRCLE, (1, 0, 0))
            world.add(puck, Pose2D(-1.5, -3.0, 0.0))  # 아래쪽 복도: 막힘 없이 동쪽으로
            puck.velocity = (0.6, 0.0)
            for _ in range(400):
                world.step(0.01)
            return puck.pose.x + 1.5

        earth = slide_distance()
        self.assertAlmostEqual(earth, 0.6**2 / (2 * 0.3 * 9.81), delta=0.02)  # v^2/(2 mu g), mu = sqrt(0.3 x 0.3)
        self.assertGreater(slide_distance(gravity=1.62), earth * 5)  # 달: 중력 6분의 1이라 6배쯤 멀리 미끄러진다
        self.assertGreater(slide_distance(ground_friction=0.03), earth * 2)  # 얼음 바닥: 훨씬 멀리
        self.assertLess(slide_distance(ground_friction=1.0), earth)  # 거친 바닥: 일찍 멈춘다

    def test_effective_friction_combines_object_and_ground(self) -> None:
        world = MazeWorld(puck_count=0, ground_friction=0.5)
        puck = Puck(PuckShape.CIRCLE, (1, 0, 0))
        puck.friction = 0.2
        self.assertAlmostEqual(world.friction_with_ground(puck), math.sqrt(0.2 * 0.5))

    def test_non_movable_object_is_not_pushed(self) -> None:
        world = MazeWorld(puck_count=0)
        wall_like, puck = Puck(PuckShape.CIRCLE, (1, 0, 0)), Puck(PuckShape.CIRCLE, (0, 1, 0))
        wall_like.movable = False
        world.add(wall_like, Pose2D(0.0, 0.0, 0.0))
        world.add(puck, Pose2D(0.05, 0.0, 0.0))
        world.step(0.1)
        self.assertEqual((wall_like.pose.x, wall_like.pose.y), (0.0, 0.0))
        self.assertGreaterEqual(puck.pose.x, wall_like.radius + puck.radius - 1e-6)

    def test_reality_side_does_not_depend_on_kros_nodes(self) -> None:
        root = pathlib.Path(__file__).parent.parent
        banned = re.compile(r"^\s*(from|import)\s+kros\.(node|context|topic|service|action)\b|^\s*from\s+kros\s+import|\bimport Node\b", re.M)
        for pkg in ("worlds", "interactions", "viewer"):  # worlds에는 robots도 들어 있다
            for f in (root / pkg).rglob("*.py"):
                self.assertIsNone(banned.search(f.read_text()), f"{f}는 kros의 Node/Context 쪽을 import한다")

    def test_only_the_viewer_depends_on_panda3d(self) -> None:
        root = pathlib.Path(__file__).parent.parent
        banned = re.compile(r"^\s*(from|import)\s+(panda3d|direct)\b", re.M)
        for pkg in ("worlds", "interactions", "node", "topic", "service", "action", "msgs"):
            for f in (root / pkg).rglob("*.py"):
                self.assertIsNone(banned.search(f.read_text()), f"{f}는 panda3d를 import한다 (화면은 viewer/ 에만)")
        for name in ("snapshot.py", "snapshot_recorder.py"):  # viewer/ 안이어도 스냅샷과 기록기는 panda3d 없이 돈다
            self.assertIsNone(banned.search((root / "viewer" / name).read_text()), f"viewer/{name}는 panda3d를 import하면 안 된다")

    def test_world_knows_nothing_about_robot_or_sensors(self) -> None:
        root = pathlib.Path(__file__).parent.parent / "worlds"
        for f in root.rglob("*.py"):
            if {"robots", "hardwares"} & set(f.relative_to(root).parts):  # 로봇과 하드웨어 부품은 월드 안에 놓이는 것들: 월드 쪽 파일만 검사
                continue
            body = f.read_text()
            for banned in ("LaserScan", "Twist", "kros.worlds.robots", "kros.viewer"):
                self.assertNotIn(banned, body, f"{f}: {banned}")


class TestBatchRays(unittest.TestCase):
    def test_ray_distances_equals_individual_rays(self) -> None:
        world = MazeWorld(seed=2)
        angles = [i * math.pi / 36 for i in range(72)]
        batch = world.ray_distances(0.3, 0.2, angles, 0.05)
        single = [world.ray_distance(0.3, 0.2, a, 0.05) for a in angles]
        self.assertEqual(len(batch), len(single))
        for b, s in zip(batch, single):
            self.assertAlmostEqual(b, s) if math.isfinite(s) else self.assertEqual(b, s)

    def test_batch_matches_single_for_many_poses_and_heights(self) -> None:
        world = MazeWorld(seed=3)
        angles = [i * 2 * math.pi / 90 for i in range(90)]
        for x, y in ((0.0, 0.0), (-1.5, -2.7), (1.6, 0.5), (0.3, 2.2), (-2.0, 3.0)):
            for height in (0.0, 0.132, 0.2, 0.3):
                batch = world.ray_distances(x, y, angles, height)
                for a, b in zip(angles, batch):
                    s = world.ray_distance(x, y, a, height)
                    self.assertAlmostEqual(b, s, msg=f"{(x, y, a, height)}") if math.isfinite(s) else self.assertEqual(b, s, f"{(x, y, a, height)}")


if __name__ == "__main__":
    unittest.main()
