import threading
import unittest

from kros.context import Context
from kros.nodes import Node
from kros.topic import QOS_DEFAULT, QOS_LATCHED, QOS_SENSOR_DATA, Durability, QoS, TopicExecutor


def latched(depth: int) -> QoS:
    return QoS(depth=depth, durability=Durability.TRANSIENT_LOCAL)


class TestQoSValue(unittest.TestCase):
    def test_depth_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            QoS(depth=0)
        with self.assertRaises(ValueError):
            QoS(depth=-3)

    def test_transient_local_needs_a_depth(self) -> None:
        with self.assertRaises(ValueError):
            QoS(durability=Durability.TRANSIENT_LOCAL)  # 몇 개를 기억할지 모르면 무제한으로 쌓일 수 있다

    def test_presets(self) -> None:
        self.assertEqual((QOS_DEFAULT.depth, QOS_DEFAULT.durability), (None, Durability.VOLATILE))
        self.assertEqual((QOS_LATCHED.depth, QOS_LATCHED.durability), (1, Durability.TRANSIENT_LOCAL))
        self.assertEqual((QOS_SENSOR_DATA.depth, QOS_SENSOR_DATA.durability), (5, Durability.VOLATILE))


class TestLatch(unittest.TestCase):
    def setUp(self) -> None:
        self.m = TopicExecutor()
        self.addCleanup(self.m.shutdown)

    def late(self, qos: QoS, topic: str = "x") -> list[int]:
        got: list[int] = []
        self.m.subscribe(topic, int, got.append, qos)
        self.m.join()
        return got

    def test_late_subscriber_gets_the_last_latched_message(self) -> None:
        for v in (1, 2, 3):
            self.m.publish("x", v, QOS_LATCHED)
        self.assertEqual(self.late(QOS_LATCHED), [3])  # 마지막 하나만

    def test_depth_keeps_the_last_n_in_order(self) -> None:
        for v in range(1, 6):
            self.m.publish("x", v, latched(3))
        self.assertEqual(self.late(latched(3)), [3, 4, 5])

    def test_history_comes_before_new_messages(self) -> None:
        self.m.publish("x", 1, QOS_LATCHED)
        got: list[int] = []
        self.m.subscribe("x", int, got.append, latched(10))  # 구독의 depth가 1이면 연달아 온 것 중 오래된 것이 버려진다
        self.m.publish("x", 2, QOS_LATCHED)
        self.m.publish("x", 3)
        self.m.join()
        self.assertEqual(got, [1, 2, 3])

    def test_volatile_subscriber_gets_nothing_from_history(self) -> None:
        self.m.publish("x", 1, QOS_LATCHED)
        self.assertEqual(self.late(QOS_DEFAULT), [])  # 구독 쪽도 TRANSIENT_LOCAL이어야 받는다 (ROS와 같은 규칙)

    def test_volatile_publisher_leaves_nothing_to_remember(self) -> None:
        self.m.publish("x", 1)
        self.assertEqual(self.late(QOS_LATCHED), [])  # 발행 쪽도 TRANSIENT_LOCAL이어야 기억된다

    def test_subscriber_already_there_gets_each_message_once(self) -> None:
        got: list[int] = []
        self.m.subscribe("x", int, got.append, latched(10))
        for v in (1, 2, 3):
            self.m.publish("x", v, QOS_LATCHED)
        self.m.join()
        self.assertEqual(got, [1, 2, 3])  # 기억해 둔 것이 한 번 더 오지 않는다

    def test_every_late_subscriber_gets_it(self) -> None:
        self.m.publish("x", 7, QOS_LATCHED)
        a, b = self.late(QOS_LATCHED), self.late(QOS_LATCHED)
        self.assertEqual((a, b), ([7], [7]))

    def test_each_topic_remembers_on_its_own(self) -> None:
        self.m.publish("a", 1, QOS_LATCHED)
        self.m.publish("b", 2, QOS_LATCHED)
        self.assertEqual((self.late(QOS_LATCHED, "a"), self.late(QOS_LATCHED, "b")), ([1], [2]))

    def test_subscriber_with_smaller_depth_never_loses_the_newest_history(self) -> None:
        for v in range(1, 5):
            self.m.publish("x", v, latched(4))
        got: list[int] = []
        sub = self.m.subscribe("x", int, got.append, latched(1))  # 기억된 4개가 한꺼번에 들어와 큐(1개)가 넘친다
        self.m.join()
        self.assertEqual(got[-1], 4)  # 가장 최신은 항상 남는다 (워커가 먼저 몇 개를 꺼냈는지는 타이밍이라 개수는 고정하지 않는다)
        self.assertEqual(len(got) + sub.dropped, 4)  # 받았거나 버렸거나: 사라진 것은 없다
        self.assertEqual(got, sorted(got))

    def test_topic_type_is_still_locked_for_latched_topics(self) -> None:
        self.m.publish("x", 1, QOS_LATCHED)
        with self.assertRaises(TypeError):
            self.m.subscribe("x", str, lambda s: None, QOS_LATCHED)
        with self.assertRaises(TypeError):
            self.m.publish("x", "no", QOS_LATCHED)

    def test_changing_the_publishers_depth_keeps_recent_messages(self) -> None:
        for v in (1, 2):
            self.m.publish("x", v, latched(2))
        self.m.publish("x", 3, latched(3))  # 더 깊게 기억하기로 했다
        self.assertEqual(self.late(latched(3)), [1, 2, 3])

    def test_shutdown_forgets_history(self) -> None:
        self.m.publish("x", 1, QOS_LATCHED)
        self.m.shutdown()
        self.assertEqual(self.m._history, {})

    def test_late_joiner_never_loses_or_repeats_messages_under_concurrency(self) -> None:
        """발행이 계속되는 중에 구독해도 기억된 마지막 것부터 끊김/중복 없이 이어서 받는다."""
        total = 2000
        started = threading.Event()

        def publisher() -> None:
            for v in range(1, total + 1):
                self.m.publish("x", v, QOS_LATCHED)
                if v == 50:
                    started.set()

        thread = threading.Thread(target=publisher)
        thread.start()
        started.wait(5)
        got: list[int] = []
        self.m.subscribe("x", int, got.append, latched(total + 10))  # 발행이 한창일 때 구독 (큐가 넘쳐 버려지지 않게 깊게)
        thread.join()
        self.m.join()
        self.assertTrue(got, "아무것도 못 받았다")
        self.assertEqual(got, list(range(got[0], total + 1)))  # 첫 값부터 마지막까지 빠짐도 중복도 없다


class TestBoundedQueue(unittest.TestCase):
    def setUp(self) -> None:
        self.m = TopicExecutor()
        self.addCleanup(self.m.shutdown)

    def blocked_subscriber(self, qos: QoS) -> tuple[list[int], threading.Event, threading.Event]:
        got: list[int] = []
        entered, release = threading.Event(), threading.Event()

        def callback(v: int) -> None:
            entered.set()
            release.wait(5)
            got.append(v)

        self.m.subscribe("x", int, callback, qos)
        return got, entered, release

    def test_slow_subscriber_skips_old_messages_when_the_queue_is_full(self) -> None:
        got, entered, release = self.blocked_subscriber(QoS(depth=3))
        self.m.publish("x", 0)
        self.assertTrue(entered.wait(5))  # 0을 처리하느라 막혀 있다
        for v in range(1, 11):
            self.m.publish("x", v)
        release.set()
        self.m.join()
        self.assertEqual(got, [0, 8, 9, 10])  # 실행 중이던 것 + 큐에는 최신 3개만 남았다

    def test_dropped_messages_are_counted_as_a_fact(self) -> None:
        got: list[int] = []
        entered, release = threading.Event(), threading.Event()

        def callback(v: int) -> None:
            entered.set()
            release.wait(5)
            got.append(v)

        sub = self.m.subscribe("x", int, callback, QoS(depth=2))
        self.m.publish("x", 0)
        self.assertTrue(entered.wait(5))
        for v in range(1, 8):
            self.m.publish("x", v)
        release.set()
        self.m.join()
        self.assertEqual((sub.dropped, got), (5, [0, 6, 7]))

    def test_depth_one_always_keeps_the_latest(self) -> None:
        got, entered, release = self.blocked_subscriber(QoS(depth=1))
        self.m.publish("x", 0)
        self.assertTrue(entered.wait(5))
        for v in range(1, 6):
            self.m.publish("x", v)
        release.set()
        self.m.join()
        self.assertEqual(got, [0, 5])

    def test_join_does_not_hang_when_messages_were_dropped(self) -> None:
        got, entered, release = self.blocked_subscriber(QoS(depth=1))
        self.m.publish("x", 0)
        self.assertTrue(entered.wait(5))
        for v in range(100):
            self.m.publish("x", v)
        release.set()
        self.m.join()  # 버려진 메시지도 끝난 것으로 세어야 여기서 안 막힌다
        self.assertEqual(len(got), 2)

    def test_default_qos_is_unbounded_and_drops_nothing(self) -> None:
        got, entered, release = self.blocked_subscriber(QOS_DEFAULT)
        self.m.publish("x", 0)
        self.assertTrue(entered.wait(5))
        for v in range(1, 301):
            self.m.publish("x", v)
        release.set()
        self.m.join()
        self.assertEqual(got, list(range(301)))  # 기본은 지금까지처럼 아무것도 버리지 않는다

    def test_a_slow_bounded_subscriber_does_not_affect_a_fast_one(self) -> None:
        _, entered, release = self.blocked_subscriber(QoS(depth=1))
        fast: list[int] = []
        self.m.subscribe("x", int, fast.append)
        self.m.publish("x", 0)
        self.assertTrue(entered.wait(5))
        for v in range(1, 50):
            self.m.publish("x", v)
        release.set()
        self.m.join()
        self.assertEqual(fast, list(range(50)))

    def test_order_is_kept_among_the_messages_that_survive(self) -> None:
        got, entered, release = self.blocked_subscriber(QoS(depth=4))
        self.m.publish("x", 0)
        self.assertTrue(entered.wait(5))
        for v in range(1, 30):
            self.m.publish("x", v)
        release.set()
        self.m.join()
        self.assertEqual(got, sorted(got))
        self.assertEqual(got[-4:], [26, 27, 28, 29])

    def test_unsubscribe_discards_queued_messages_and_join_returns(self) -> None:
        got, entered, release = self.blocked_subscriber(QOS_DEFAULT)
        sub = self.m.subscribe("y", int, lambda v: None)
        self.m.unsubscribe(sub)
        self.m.publish("x", 0)
        self.assertTrue(entered.wait(5))
        for v in range(1, 5):
            self.m.publish("x", v)
        release.set()
        self.m.join()
        self.assertEqual(got, [0, 1, 2, 3, 4])


class TestNodeQoS(unittest.TestCase):
    def test_node_passes_qos_through(self) -> None:
        ctx = Context()
        self.addCleanup(ctx.shutdown)
        publisher, late = Node(ctx, "pub"), Node(ctx, "late")
        publisher.publish("static", 42, QOS_LATCHED)
        got: list[int] = []
        late.subscribe("static", int, got.append, QOS_LATCHED)
        ctx.topic_executor.join()
        self.assertEqual(got, [42])  # 늦게 뜬 노드도 한 번만 낸 정보를 받는다

    def test_default_node_calls_behave_as_before(self) -> None:
        ctx = Context()
        self.addCleanup(ctx.shutdown)
        a, b = Node(ctx, "a"), Node(ctx, "b")
        a.publish("t", 1)
        got: list[int] = []
        b.subscribe("t", int, got.append)
        ctx.topic_executor.join()
        self.assertEqual(got, [])  # 기본은 휘발성: 구독 전에 나간 것은 못 받는다


if __name__ == "__main__":
    unittest.main()
