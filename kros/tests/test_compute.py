import math
import os
import unittest
from unittest import mock

from kros.worlds.compute import GPU_MIN_WORK, Compute
from kros.worlds.maze.maze_world import MazeWorld

ANGLES = [i * 2 * math.pi / 72 for i in range(72)]


def same(a: list[float], b: list[float]) -> bool:
    return len(a) == len(b) and all(math.isclose(x, y, abs_tol=1e-9) if math.isfinite(y) else x == y for x, y in zip(a, b))


def reference(world: MazeWorld, angles: list[float]) -> list[float]:
    return [world.ray_distance(0.3, 0.2, a, 0.1) for a in angles]


class FakeBrokenGpu:
    """GPU가 있는 척하지만 계산하면 실패하는 가짜 cupy."""

    ndarray = type("ndarray", (), {})

    def asarray(self, *args: object, **kwargs: object) -> object:
        raise RuntimeError("CUDA 오류")


class TestComputeFallback(unittest.TestCase):
    def test_pure_python_when_nothing_is_installed(self) -> None:
        compute = Compute(numpy=None, cupy=None)  # 라즈베리파이 같은 곳
        self.assertEqual(compute.name, "python")
        self.assertIsNone(compute.pick(10**9))
        world = MazeWorld(seed=1, compute_backend=compute)
        self.assertTrue(same(world.ray_distances(0.3, 0.2, ANGLES, 0.1), reference(world, ANGLES)))

    def test_python_mode_forced(self) -> None:
        self.assertIsNone(Compute("python").pick(10**9))

    def test_cpu_numpy_matches_reference(self) -> None:
        compute = Compute("cpu")
        if compute.numpy is None:
            self.skipTest("numpy 없음 (core 프로필)")
        self.assertEqual(compute.name, "cpu(numpy)")
        world = MazeWorld(seed=1, compute_backend=compute)
        self.assertTrue(same(world.ray_distances(0.3, 0.2, ANGLES, 0.1), reference(world, ANGLES)))

    def test_gpu_failure_falls_back_to_cpu_once_and_stays_there(self) -> None:
        compute = Compute("gpu", cupy=FakeBrokenGpu())
        if compute.numpy is None:
            self.skipTest("numpy 없음 (core 프로필)")
        world = MazeWorld(seed=1, compute_backend=compute)
        with self.assertLogs("kros.worlds.compute", level="WARNING"):
            result = world.ray_distances(0.3, 0.2, ANGLES, 0.1)
        self.assertTrue(same(result, reference(world, ANGLES)))  # 실패해도 같은 결과
        self.assertIsNone(compute.cupy)  # 이후엔 GPU를 다시 시도하지 않는다
        self.assertEqual(compute.name, "cpu(numpy)")
        self.assertTrue(same(world.ray_distances(0.3, 0.2, ANGLES, 0.1), reference(world, ANGLES)))

    def test_gpu_requested_but_not_available_uses_cpu(self) -> None:
        compute = Compute("gpu", cupy=None)
        if compute.numpy is None:
            self.skipTest("numpy 없음 (core 프로필)")
        self.assertEqual(compute.name, "cpu(numpy)")
        self.assertIs(compute.pick(10**9), compute.numpy)

    def test_auto_uses_gpu_only_for_big_work(self) -> None:
        cpu, gpu = object(), object()
        compute = Compute("auto", numpy=cpu, cupy=gpu)
        self.assertIs(compute.pick(GPU_MIN_WORK - 1), cpu)  # 작으면 CPU가 더 빠르다
        self.assertIs(compute.pick(GPU_MIN_WORK), gpu)

    def test_invalid_mode_raises(self) -> None:
        with self.assertRaises(ValueError):
            Compute("tpu")

    def test_mode_from_environment(self) -> None:
        with mock.patch.dict(os.environ, {"KROS_COMPUTE": "python"}):
            self.assertEqual(Compute().name, "python")


class TestRealGpu(unittest.TestCase):
    def setUp(self) -> None:
        self.compute = Compute("gpu")
        if self.compute.cupy is None:
            self.skipTest("cupy/CUDA GPU 없음")

    def test_gpu_matches_cpu_small_and_big(self) -> None:
        cpu = MazeWorld(seed=1, compute_backend=Compute("cpu"))
        gpu = MazeWorld(seed=1, compute_backend=self.compute)
        for n in (72, 5000):
            angles = [i * 2 * math.pi / n for i in range(n)]
            self.assertTrue(same(gpu.ray_distances(0.3, 0.2, angles, 0.1), cpu.ray_distances(0.3, 0.2, angles, 0.1)), n)


if __name__ == "__main__":
    unittest.main()
