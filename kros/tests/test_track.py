import math
import unittest

from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.maze.track import WALLS, Track


class TestTrack(unittest.TestCase):
    def test_whole_track_is_one_static_world_object(self) -> None:
        world = MazeWorld(puck_count=0)
        tracks = [o for o in world.objects if isinstance(o, Track)]
        self.assertEqual(len(tracks), 1)
        self.assertEqual(len(world.objects), 1)  # 벽 20개가 따로 있지 않다
        self.assertEqual(len(tracks[0].boxes), len(WALLS))
        self.assertTrue(tracks[0].static and math.isinf(tracks[0].mass))

    def test_ray_hits_the_outer_wall_and_batch_agrees(self) -> None:
        world = MazeWorld(puck_count=0)
        single = world.ray_distance(0.0, 0.0, math.pi, 0.1)
        self.assertAlmostEqual(single, 2.24 - 0.025, places=2)  # 왼쪽 바깥 벽 안쪽 면까지
        angles = [i * math.tau / 36 for i in range(36)]
        batch = world.ray_distances(0.0, 0.0, angles, 0.1)
        for a, b in zip(batch, (world.ray_distance(0.0, 0.0, t, 0.1) for t in angles)):
            self.assertAlmostEqual(a, b)

    def test_circle_cannot_enter_a_wall(self) -> None:
        world = MazeWorld(puck_count=0)
        x, y = world.push_out(-2.24, 0.0, 0.1)  # 바깥 벽 한가운데에 놓으면 밀려난다
        self.assertGreater(abs(x + 2.24), 0.1)


if __name__ == "__main__":
    unittest.main()
