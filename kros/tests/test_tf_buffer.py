import math
import threading
import unittest

from kros.msgs.geometry_msgs import Quaternion, Transform, Vector3
from kros.tf import ConnectivityException, ExtrapolationException, LookupException, TransformBuffer, TransformException
from kros.tf.transform_math import from_axis_angle, from_rpy


def shift(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Transform:
    return Transform(Vector3(x, y, z))


def turn(yaw: float, x: float = 0.0, y: float = 0.0) -> Transform:
    return Transform(Vector3(x, y, 0.0), from_axis_angle((0, 0, 1), yaw))


def xyz(buffer: TransformBuffer, target: str, source: str, time: float | None = None) -> tuple[float, float, float]:
    t = buffer.lookup_transform(target, source, time).transform.translation
    return round(t.x, 9), round(t.y, 9), round(t.z, 9)


class TestStaticTree(unittest.TestCase):
    def setUp(self) -> None:
        self.buffer = TransformBuffer()
        # base_footprint -> base_link -> base_scan, base_link -> imu
        self.buffer.set_transform("base_footprint", "base_link", 0.0, shift(z=0.010), is_static=True)
        self.buffer.set_transform("base_link", "base_scan", 0.0, shift(-0.064, 0, 0.122), is_static=True)
        self.buffer.set_transform("base_link", "imu", 0.0, shift(0, 0, 0.068), is_static=True)

    def test_chain_lookup_adds_up(self) -> None:
        self.assertEqual(xyz(self.buffer, "base_footprint", "base_scan"), (-0.064, 0.0, 0.132))  # 0.010 + 0.122

    def test_reverse_lookup_is_the_inverse(self) -> None:
        self.assertEqual(xyz(self.buffer, "base_scan", "base_footprint"), (0.064, 0.0, -0.132))

    def test_siblings_meet_at_their_common_ancestor(self) -> None:
        self.assertEqual(xyz(self.buffer, "imu", "base_scan"), (-0.064, 0.0, 0.054))  # imu에서 본 scan: (-0.064, 0, 0.122-0.068)

    def test_result_names_target_as_parent_and_source_as_child(self) -> None:
        stamped = self.buffer.lookup_transform("base_footprint", "base_scan")
        self.assertEqual((stamped.header.frame_id, stamped.child_frame_id), ("base_footprint", "base_scan"))

    def test_same_frame_is_identity(self) -> None:
        self.assertEqual(xyz(self.buffer, "imu", "imu"), (0.0, 0.0, 0.0))

    def test_static_transforms_do_not_depend_on_time(self) -> None:
        self.assertEqual(xyz(self.buffer, "base_footprint", "base_scan", 123.0), (-0.064, 0.0, 0.132))

    def test_all_frames(self) -> None:
        self.assertEqual(self.buffer.all_frames(), ["base_footprint", "base_link", "base_scan", "imu"])

    def test_rotation_in_the_chain_rotates_the_offsets_below_it(self) -> None:
        buffer = TransformBuffer()
        buffer.set_transform("map", "robot", 0.0, turn(math.pi / 2, x=1.0), is_static=True)  # 로봇은 (1,0)에서 왼쪽(+y)을 본다
        buffer.set_transform("robot", "lidar", 0.0, shift(x=0.5), is_static=True)  # 라이다는 로봇 앞 0.5m
        self.assertEqual(xyz(buffer, "map", "lidar"), (1.0, 0.5, 0.0))  # 로봇이 +y를 보니 라이다는 (1, 0.5)

    def test_rpy_urdf_origin_is_respected(self) -> None:
        buffer = TransformBuffer()
        buffer.set_transform("base", "wheel", 0.0, Transform(Vector3(0, 0.144, 0.023), from_rpy(-1.57, 0, 0)), is_static=True)
        axis_in_base = buffer.lookup_transform("base", "wheel").transform.rotation
        from kros.tf.transform_math import rotate

        spin = rotate(axis_in_base, Vector3(0, 0, 1))
        self.assertAlmostEqual(spin.y, 1.0, places=3)  # 바퀴 회전축(z)이 로봇의 +y를 향한다


class TestErrors(unittest.TestCase):
    def setUp(self) -> None:
        self.buffer = TransformBuffer()
        self.buffer.set_transform("a", "b", 0.0, shift(1), is_static=True)
        self.buffer.set_transform("x", "y", 0.0, shift(1), is_static=True)

    def test_unknown_frame(self) -> None:
        with self.assertRaises(LookupException):
            self.buffer.lookup_transform("a", "nope")
        with self.assertRaises(LookupException):
            self.buffer.lookup_transform("nope", "a")

    def test_known_but_not_connected(self) -> None:
        with self.assertRaises(ConnectivityException):
            self.buffer.lookup_transform("a", "y")

    def test_all_errors_share_a_parent_and_can_transform_wraps_them(self) -> None:
        for error in (LookupException, ConnectivityException, ExtrapolationException):
            self.assertTrue(issubclass(error, TransformException))
        self.assertTrue(self.buffer.can_transform("a", "b"))
        self.assertFalse(self.buffer.can_transform("a", "nope"))
        self.assertFalse(self.buffer.can_transform("a", "y"))

    def test_loops_are_rejected(self) -> None:
        with self.assertRaises(ConnectivityException):
            self.buffer.set_transform("b", "a", 0.0, shift(1), is_static=True)  # a가 b의 부모인데 b가 a의 부모가 되려 한다
        self.assertEqual(xyz(self.buffer, "a", "b"), (1.0, 0.0, 0.0))  # 거절돼도 기존 트리는 그대로

    def test_a_frame_cannot_be_its_own_parent(self) -> None:
        with self.assertRaises(ValueError):
            self.buffer.set_transform("a", "a", 0.0, shift())

    def test_bad_cache_time(self) -> None:
        with self.assertRaises(ValueError):
            TransformBuffer(cache_time=0)


class TestDynamic(unittest.TestCase):
    def setUp(self) -> None:
        self.buffer = TransformBuffer(cache_time=10.0)
        for t, x in ((1.0, 0.0), (2.0, 1.0), (3.0, 3.0)):
            self.buffer.set_transform("odom", "base", t, shift(x))

    def test_exact_sample_times(self) -> None:
        self.assertEqual(xyz(self.buffer, "odom", "base", 2.0), (1.0, 0.0, 0.0))

    def test_between_samples_is_interpolated(self) -> None:
        self.assertEqual(xyz(self.buffer, "odom", "base", 1.5), (0.5, 0.0, 0.0))
        self.assertEqual(xyz(self.buffer, "odom", "base", 2.75), (2.5, 0.0, 0.0))

    def test_latest_when_time_is_not_given(self) -> None:
        stamped = self.buffer.lookup_transform("odom", "base")
        self.assertEqual((stamped.transform.translation.x, stamped.header.stamp), (3.0, 3.0))

    def test_outside_the_history_is_an_extrapolation_error(self) -> None:
        with self.assertRaisesRegex(ExtrapolationException, "1.000~3.000"):
            self.buffer.lookup_transform("odom", "base", 3.5)  # 미래
        with self.assertRaises(ExtrapolationException):
            self.buffer.lookup_transform("odom", "base", 0.5)  # 과거

    def test_rotation_is_interpolated_spherically(self) -> None:
        buffer = TransformBuffer()
        buffer.set_transform("odom", "base", 0.0, turn(0.0))
        buffer.set_transform("odom", "base", 1.0, turn(math.pi / 2))
        mid = buffer.lookup_transform("odom", "base", 0.5).transform.rotation
        expected = from_axis_angle((0, 0, 1), math.pi / 4)
        self.assertAlmostEqual(mid.z, expected.z)
        self.assertAlmostEqual(mid.w, expected.w)

    def test_samples_can_arrive_out_of_order(self) -> None:
        buffer = TransformBuffer()
        for t in (3.0, 1.0, 2.0):
            buffer.set_transform("odom", "base", t, shift(t))
        self.assertEqual(xyz(buffer, "odom", "base", 1.5), (1.5, 0.0, 0.0))

    def test_same_stamp_overwrites(self) -> None:
        self.buffer.set_transform("odom", "base", 2.0, shift(10.0))
        self.assertEqual(xyz(self.buffer, "odom", "base", 2.0), (10.0, 0.0, 0.0))

    def test_old_samples_are_dropped_after_cache_time(self) -> None:
        buffer = TransformBuffer(cache_time=2.0)
        for t in range(0, 10):
            buffer.set_transform("odom", "base", float(t), shift(float(t)))
        with self.assertRaises(ExtrapolationException):
            buffer.lookup_transform("odom", "base", 5.0)  # 가장 최근(9) 기준 2초보다 오래됐다
        self.assertEqual(xyz(buffer, "odom", "base", 8.0), (8.0, 0.0, 0.0))
        buffer.set_transform("odom", "base", 1.0, shift(1.0))  # 보관 범위보다 오래된 새 데이터는 무시한다
        with self.assertRaises(ExtrapolationException):
            buffer.lookup_transform("odom", "base", 1.0)

    def test_dynamic_over_static_chain_uses_the_common_latest_time(self) -> None:
        self.buffer.set_transform("base", "lidar", 0.0, shift(z=0.2), is_static=True)
        stamped = self.buffer.lookup_transform("odom", "lidar")
        self.assertEqual((stamped.transform.translation.x, stamped.transform.translation.z, stamped.header.stamp), (3.0, 0.2, 3.0))

    def test_two_moving_frames_use_the_oldest_of_their_latest_stamps(self) -> None:
        buffer = TransformBuffer()
        for t in (1.0, 2.0, 3.0):
            buffer.set_transform("map", "a", t, shift(t))
        for t in (1.0, 2.0):
            buffer.set_transform("map", "b", t, shift(-t))  # b는 2초까지만 왔다
        stamped = buffer.lookup_transform("a", "b")
        self.assertEqual(stamped.header.stamp, 2.0)  # 둘 다 값이 있는 가장 최근 시각
        self.assertAlmostEqual(stamped.transform.translation.x, -4.0)  # a(2)에서 본 b(-2)는 -4


class TestReparenting(unittest.TestCase):
    def test_changing_a_frames_parent_drops_its_old_history(self) -> None:
        buffer = TransformBuffer()
        buffer.set_transform("a", "child", 1.0, shift(1))
        buffer.set_transform("a", "sibling", 1.0, shift(2))  # a는 sibling의 부모로도 남아 있다
        with self.assertLogs("kros.tf.transform_buffer", level="WARNING"):
            buffer.set_transform("b", "child", 2.0, shift(5))
        with self.assertRaises(ConnectivityException):
            buffer.lookup_transform("sibling", "child")  # child는 이제 b 아래라 a 아래의 sibling과 이어져 있지 않다
        self.assertEqual(xyz(buffer, "b", "child", 2.0), (5.0, 0.0, 0.0))
        with self.assertRaises(ExtrapolationException):
            buffer.lookup_transform("b", "child", 1.0)  # 옛 부모 아래에서 쌓은 시각 1.0의 이력은 버려졌다

    def test_static_replaces_dynamic_and_dynamic_replaces_static(self) -> None:
        buffer = TransformBuffer()
        buffer.set_transform("a", "b", 1.0, shift(1))
        buffer.set_transform("a", "b", 0.0, shift(7), is_static=True)
        self.assertEqual(xyz(buffer, "a", "b", 99.0), (7.0, 0.0, 0.0))  # 정적이 대체했다 (시각과 상관없다)
        buffer.set_transform("a", "b", 5.0, shift(9))
        self.assertEqual(xyz(buffer, "a", "b", 5.0), (9.0, 0.0, 0.0))


class TestMisc(unittest.TestCase):
    def test_non_unit_quaternions_are_normalized(self) -> None:
        buffer = TransformBuffer()
        buffer.set_transform("a", "b", 0.0, Transform(Vector3(), Quaternion(0, 0, 3, 3)), is_static=True)
        q = buffer.lookup_transform("a", "b").transform.rotation
        self.assertAlmostEqual(q.z**2 + q.w**2, 1.0)

    def test_clear(self) -> None:
        buffer = TransformBuffer()
        buffer.set_transform("a", "b", 0.0, shift(1), is_static=True)
        buffer.clear()
        self.assertEqual(buffer.all_frames(), [])

    def test_concurrent_writers_and_readers_do_not_corrupt_it(self) -> None:
        buffer = TransformBuffer(cache_time=1000.0)
        buffer.set_transform("world", "robot", 0.0, shift(0.0))
        errors: list[Exception] = []

        def writer() -> None:
            for i in range(1, 400):
                buffer.set_transform("world", "robot", float(i), shift(float(i)))

        def reader() -> None:
            for _ in range(400):
                try:
                    stamped = buffer.lookup_transform("world", "robot")
                    assert abs(stamped.transform.translation.x - stamped.header.stamp) < 1e-9  # 같은 시각의 값이다
                except ExtrapolationException:
                    pass
                except Exception as error:  # noqa: BLE001
                    errors.append(error)

        threads = [threading.Thread(target=writer)] + [threading.Thread(target=reader) for _ in range(3)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
