import math
import threading
import time
import unittest

from kros import Kros
from kros.context import Context
from kros.msgs.geometry_msgs import Point, PointStamped, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Vector3
from kros.msgs.std_msgs import Header
from kros.nodes import Node
from kros.nodes import TfEcho
from kros.tf import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformBroadcaster,
    view_frames,
)
from kros.tf.transform_math import from_axis_angle, from_rpy, to_rpy
from kros.timer import SimulatedTimeSource


def shift(x: float = 0.0) -> Transform:
    return Transform(Vector3(x, 0, 0))


class TestWaiting(unittest.TestCase):
    def test_no_timeout_fails_at_once(self) -> None:
        started = time.monotonic()
        with self.assertRaises(LookupException):
            Buffer().lookup_transform("a", "b")
        self.assertLess(time.monotonic() - started, 0.1)

    def test_lookup_waits_until_the_transform_arrives(self) -> None:
        buffer = Buffer()
        threading.Timer(0.1, lambda: buffer.set_transform("a", "b", 1.0, shift(2.0))).start()
        started = time.monotonic()
        t = buffer.lookup_transform("a", "b", timeout=2.0)
        self.assertEqual(t.transform.translation.x, 2.0)
        self.assertLess(time.monotonic() - started, 1.0)  # 시간 끝까지 기다리지 않고 도착 즉시 돌아온다

    def test_timeout_raises_the_underlying_exception(self) -> None:
        buffer = Buffer()
        buffer.set_transform("a", "b", 1.0, shift())
        started = time.monotonic()
        with self.assertRaises(ExtrapolationException):
            buffer.lookup_transform("a", "b", 5.0, timeout=0.2)
        self.assertGreaterEqual(time.monotonic() - started, 0.19)

    def test_can_transform_waits_too(self) -> None:
        buffer = Buffer()
        threading.Timer(0.1, lambda: buffer.set_transform("a", "b", 1.0, shift())).start()
        self.assertTrue(buffer.can_transform("a", "b", timeout=2.0))
        self.assertFalse(buffer.can_transform("a", "zzz", timeout=0.05))

    def test_waits_for_a_future_time(self) -> None:
        buffer = Buffer()
        buffer.set_transform("a", "b", 1.0, shift(1.0))
        threading.Timer(0.1, lambda: buffer.set_transform("a", "b", 3.0, shift(3.0))).start()
        self.assertAlmostEqual(buffer.lookup_transform("a", "b", 2.0, timeout=2.0).transform.translation.x, 2.0)


class TestFrameDumps(unittest.TestCase):
    def setUp(self) -> None:
        self.buffer = Buffer()
        self.buffer.set_transform("odom", "base", 1.0, shift())
        self.buffer.set_transform("odom", "base", 2.0, shift(1.0))
        self.buffer.set_transform("odom", "base", 3.0, shift(2.0))
        self.buffer.set_transform("base", "scan", 0.0, shift(), is_static=True)

    def test_as_string_lists_every_frame_with_its_parent(self) -> None:
        self.assertEqual(
            self.buffer.all_frames_as_string(),
            "Frame base exists with parent odom.\nFrame odom exists with parent NO_PARENT.\nFrame scan exists with parent base.\n",
        )

    def test_as_yaml_has_rate_and_range_of_the_history(self) -> None:
        yaml = self.buffer.all_frames_as_yaml()
        self.assertIn("base:\n  parent: 'odom'\n  broadcaster: 'tf'\n  rate: 1.000\n  most_recent_transform: 3.000000\n  oldest_transform: 1.000000\n  buffer_length: 2.000", yaml)
        self.assertIn("scan:\n  parent: 'base'\n  broadcaster: 'static'", yaml)

    def test_empty_buffer_dumps_nothing(self) -> None:
        self.assertEqual((Buffer().all_frames_as_string(), Buffer().all_frames_as_yaml()), ("", ""))

    def test_view_frames_draws_the_tree(self) -> None:
        self.buffer.set_transform("base", "imu", 0.0, shift(), is_static=True)
        self.assertEqual(view_frames(self.buffer), "odom\n  base\n    imu\n    scan\n")
        self.assertEqual(view_frames(Buffer()), "")


class TestTransformStamped(unittest.TestCase):
    def setUp(self) -> None:
        self.buffer = Buffer()  # base에서 본 laser: 앞으로 1, 위로 0 + 90도 왼쪽으로 돌아 있다
        self.buffer.set_transform("base", "laser", 0.0, Transform(Vector3(1, 0, 0), from_axis_angle((0, 0, 1), math.pi / 2)), is_static=True)

    def test_point_is_moved_to_the_target_frame(self) -> None:
        out = self.buffer.transform(PointStamped(Header(0.0, "laser"), Point(1, 0, 0)), "base")
        assert isinstance(out, PointStamped)
        self.assertEqual(out.header.frame_id, "base")
        self.assertAlmostEqual(out.point.x, 1.0)
        self.assertAlmostEqual(out.point.y, 1.0)  # laser의 x축 방향은 base의 y축이다

    def test_pose_gets_position_and_orientation(self) -> None:
        out = self.buffer.transform(PoseStamped(Header(0.0, "laser"), Pose(Point(0, 0, 0), Quaternion())), "base")
        assert isinstance(out, PoseStamped)
        self.assertAlmostEqual(out.pose.position.x, 1.0)
        self.assertAlmostEqual(to_rpy(out.pose.orientation)[2], math.pi / 2)

    def test_stamp_selects_the_time_and_zero_means_latest(self) -> None:
        buffer = Buffer()
        buffer.set_transform("odom", "base", 1.0, shift(1.0))
        buffer.set_transform("odom", "base", 3.0, shift(3.0))

        def at(stamp: float) -> PointStamped:
            out = buffer.transform(PointStamped(Header(stamp, "base"), Point()), "odom")
            assert isinstance(out, PointStamped)
            return out

        self.assertAlmostEqual(at(2.0).point.x, 2.0)
        self.assertAlmostEqual(at(0.0).point.x, 3.0)
        with self.assertRaises(ExtrapolationException):
            at(9.0)

    def test_unknown_frame_raises(self) -> None:
        with self.assertRaises(LookupException):
            self.buffer.transform(PointStamped(Header(0.0, "nowhere"), Point()), "base")


class TestToRpy(unittest.TestCase):
    def test_round_trips_from_rpy(self) -> None:
        for rpy in [(0.1, 0.2, 0.3), (-1.0, 0.5, 2.5), (3.0, -0.4, -2.0), (0.0, 0.0, 0.0)]:
            for got, want in zip(to_rpy(from_rpy(*rpy)), rpy):
                self.assertAlmostEqual(got, want)

    def test_gimbal_lock_still_gives_the_same_rotation(self) -> None:
        for pitch in (math.pi / 2, -math.pi / 2):
            q = from_rpy(0.0, pitch, 0.7)
            r = from_rpy(*to_rpy(q))
            self.assertAlmostEqual(abs(q.x * r.x + q.y * r.y + q.z * r.z + q.w * r.w), 1.0)  # 같은 회전(부호만 다를 수 있다)


class TestTfEcho(unittest.TestCase):
    def make(self, parameters: dict[str, bool | int | float | str]) -> tuple[Kros, Node, TfEcho, SimulatedTimeSource]:
        source = SimulatedTimeSource()
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["pub"], made["echo"] = Node(ctx, "Pub"), TfEcho(ctx, parameters=parameters)
            return list(made.values())

        kros = Kros(create, time_source=source)
        self.addCleanup(kros.shutdown)
        echo = made["echo"]
        assert isinstance(echo, TfEcho)
        return kros, made["pub"], echo, source

    def tick(self, kros: Kros, source: SimulatedTimeSource, seconds: float) -> None:
        source.set(source.now() + seconds)
        kros.context.timer_manager.join()

    def test_logs_translation_and_rpy_and_warns_once_while_missing(self) -> None:
        kros, pub, echo, source = self.make({"target_frame": "odom", "source_frame": "base"})
        with self.assertLogs("kros.node.TfEcho", level="INFO") as logs:
            self.tick(kros, source, 1.0)
            self.tick(kros, source, 1.0)  # 같은 실패는 한 번만 알린다
            TransformBroadcaster(pub).send_transform(
                TransformStamped(Header(2.0, "odom"), "base", Transform(Vector3(1, 2, 3), from_axis_angle((0, 0, 1), math.pi / 2)))
            )
            kros.context.topic_manager.join()
            self.tick(kros, source, 1.0)
        text = "\n".join(logs.output)
        self.assertEqual(text.count("모른다"), 1)
        self.assertIn("Translation: [1.000, 2.000, 3.000]", text)
        self.assertIn("RPY (radian) [0.000, 0.000, 1.571]", text)

    def test_rate_must_be_positive(self) -> None:
        from kros.parameter import ParameterRejectedError

        with self.assertRaises(ParameterRejectedError):
            self.make({"rate": 0.0})

    def test_does_nothing_without_frames(self) -> None:
        kros, _, echo, source = self.make({})
        self.tick(kros, source, 3.0)  # 예외 없이 지나간다
        self.assertEqual(echo.buffer.all_frames(), [])


class TestListenerOrder(unittest.TestCase):
    def test_buffer_comes_first_like_ros(self) -> None:
        from kros.tf import TransformListener

        kros = Kros(lambda ctx: [Node(ctx, "N")], time_source=SimulatedTimeSource())
        self.addCleanup(kros.shutdown)
        buffer = Buffer()
        self.assertIs(TransformListener(buffer, kros.nodes[0]).buffer, buffer)

    def test_send_transform_has_the_ros_alias(self) -> None:
        self.assertIs(TransformBroadcaster.sendTransform, TransformBroadcaster.send_transform)
        self.assertTrue(issubclass(ConnectivityException, Exception))


if __name__ == "__main__":
    unittest.main()
