import threading
import unittest

from kros.topic import TopicManager


class TestTopicManager(unittest.TestCase):
    def setUp(self) -> None:
        self.m = TopicManager()
        self.addCleanup(self.m.shutdown)

    def test_publish_reaches_subscribers_of_topic_only(self) -> None:
        a: list[int] = []
        b: list[int] = []
        self.m.subscribe("x", int, a.append)
        self.m.subscribe("y", int, b.append)
        self.m.publish("x", 1)
        self.m.join()
        self.assertEqual((a, b), ([1], []))

    def test_publish_without_subscriber_is_noop(self) -> None:
        self.m.publish("none", 1)
        self.m.join()

    def test_topic_type_mismatch_raises(self) -> None:
        self.m.subscribe("x", int, lambda v: None)
        with self.assertRaises(TypeError):
            self.m.publish("x", "str")
        with self.assertRaises(TypeError):
            self.m.subscribe("x", str, lambda v: None)

    def test_unsubscribe_stops_delivery(self) -> None:
        got: list[int] = []
        sub = self.m.subscribe("x", int, got.append)
        self.m.publish("x", 1)
        self.m.join()
        self.m.unsubscribe(sub)
        self.m.unsubscribe(sub)  # 두 번 해도 안전
        self.m.publish("x", 2)
        self.m.join()
        self.assertEqual(got, [1])

    def test_failing_subscriber_does_not_block_others(self) -> None:
        got: list[int] = []
        self.m.subscribe("x", int, lambda v: None if 1 / 0 else None)
        self.m.subscribe("x", int, got.append)
        with self.assertLogs("kros.topic.subscription", "ERROR"):
            self.m.publish("x", 1)
            self.m.join()
        self.assertEqual(got, [1])

    def test_subscribers_run_concurrently(self) -> None:
        release = threading.Event()
        got: list[int] = []
        self.m.subscribe("x", int, lambda v: None if release.wait(5) else None)  # 첫 구독자가 막혀 있어도
        self.m.subscribe("x", int, got.append)
        self.m.publish("x", 1)
        deadline = threading.Event()
        for _ in range(100):  # 두 번째는 바로 받아야 한다
            if got:
                break
            deadline.wait(0.01)
        self.assertEqual(got, [1])
        release.set()

    def test_order_kept_per_subscriber(self) -> None:
        got: list[int] = []
        self.m.subscribe("x", int, got.append)
        for i in range(200):
            self.m.publish("x", i)
        self.m.join()
        self.assertEqual(got, list(range(200)))

    def test_publish_inside_callback_is_waited_by_join(self) -> None:
        got: list[int] = []
        self.m.subscribe("b", int, got.append)
        self.m.subscribe("a", int, lambda v: self.m.publish("b", v + 1))
        self.m.publish("a", 1)
        self.m.join()
        self.assertEqual(got, [2])
