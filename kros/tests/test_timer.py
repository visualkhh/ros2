import logging
import threading
import time
import unittest

from kros import Kros
from kros.context import Context
from kros.nodes import Node
from kros.timer import SimulatedTimeSource, SystemTimeSource, TimerManager


def make() -> tuple[SimulatedTimeSource, TimerManager]:
    source = SimulatedTimeSource()
    manager = TimerManager(source)
    return source, manager


class TestSimulatedTimeSource(unittest.TestCase):
    def test_set_and_advance(self) -> None:
        source = SimulatedTimeSource(1.0)
        self.assertEqual((source.now(), source.monotonic()), (1.0, 1.0))
        source.advance(0.5)
        source.set(3.0)
        self.assertEqual(source.now(), 3.0)

    def test_time_cannot_go_backwards(self) -> None:
        source = SimulatedTimeSource(5.0)
        source.set(5.0)  # 같은 값은 괜찮다
        with self.assertRaises(ValueError):
            source.set(4.9)
        self.assertEqual(source.now(), 5.0)

    def test_system_time_source_follows_the_wall_clock(self) -> None:
        source = SystemTimeSource()
        self.assertAlmostEqual(source.now(), time.time(), delta=0.5)
        a = source.monotonic()
        self.assertGreaterEqual(source.monotonic(), a)


class TestTimerWithSimulatedTime(unittest.TestCase):
    """시뮬레이션 시간: set()을 부른 스레드에서 바로 판정하니 결정적이다."""

    def setUp(self) -> None:
        self.source, self.manager = make()
        self.addCleanup(self.manager.shutdown)

    def advance_to(self, t: float) -> None:
        self.source.set(t)
        self.assertTrue(self.manager.join(5))

    def test_first_call_comes_after_one_period_not_before(self) -> None:
        calls: list[int] = []
        self.manager.create(0.2, lambda: calls.append(1))
        self.advance_to(0.19)
        self.assertEqual(calls, [])
        self.advance_to(0.2)
        self.assertEqual(len(calls), 1)
        self.advance_to(0.4)
        self.assertEqual(len(calls), 2)

    def test_no_drift_with_float_periods(self) -> None:
        calls: list[int] = []
        timer = self.manager.create(0.1, lambda: calls.append(1))
        for _ in range(10):
            self.source.advance(0.1)  # 0.1을 10번 더한 값은 1.0이 아니다 (부동소수점)
            self.assertTrue(self.manager.join(5))
        self.assertEqual((len(calls), timer.missed), (10, 0))

    def test_big_time_jump_calls_once_and_counts_missed_but_keeps_the_beat(self) -> None:
        calls: list[int] = []
        timer = self.manager.create(0.2, lambda: calls.append(1))
        self.advance_to(2.0)  # 주기 10번을 건너뜀
        self.assertEqual((len(calls), timer.missed), (1, 9))
        self.advance_to(2.1)
        self.assertEqual(len(calls), 1)  # 박자는 유지: 다음은 2.2
        self.advance_to(2.2)
        self.assertEqual(len(calls), 2)

    def test_callback_runs_on_the_timers_own_thread(self) -> None:
        names: list[str] = []
        self.manager.create(0.1, lambda: names.append(threading.current_thread().name))
        self.advance_to(0.1)
        self.assertEqual(len(names), 1)
        self.assertTrue(names[0].startswith("timer"))
        self.assertNotEqual(names[0], threading.current_thread().name)

    def test_exception_in_callback_is_logged_and_timer_keeps_running(self) -> None:
        calls: list[int] = []

        def callback() -> None:
            calls.append(1)
            if len(calls) == 1:
                raise RuntimeError("boom")

        timer = self.manager.create(0.1, callback)
        with self.assertLogs("kros.timer.timer_manager", level="ERROR"):
            self.advance_to(0.1)
        self.advance_to(0.2)
        self.assertEqual(len(calls), 2)
        self.assertEqual(timer.calls, 1)  # 성공한 호출만 센다

    def test_slow_callback_drops_new_calls_instead_of_piling_up(self) -> None:
        started, release = threading.Event(), threading.Event()
        calls: list[int] = []

        def slow() -> None:
            calls.append(1)
            started.set()
            release.wait(5)

        timer = self.manager.create(0.2, slow)
        self.source.set(0.2)
        self.assertTrue(started.wait(5))
        self.source.set(0.4)  # 아직 실행 중: 이번 호출은 버린다
        self.source.set(0.6)
        self.assertEqual(timer.missed, 2)
        release.set()
        self.assertTrue(self.manager.join(5))
        self.assertEqual(len(calls), 1)
        self.advance_to(0.8)
        self.assertEqual(len(calls), 2)  # 끝난 뒤에는 다시 돈다

    def test_two_timers_are_independent(self) -> None:
        fast: list[int] = []
        slow: list[int] = []
        self.manager.create(0.1, lambda: fast.append(1))
        self.manager.create(0.5, lambda: slow.append(1))
        self.advance_to(0.5)
        self.assertEqual((len(fast), len(slow)), (1, 1))  # 0.1짜리는 한꺼번에 건너뛰어 한 번
        self.advance_to(0.6)
        self.assertEqual((len(fast), len(slow)), (2, 1))

    def test_cancel_stops_the_timer(self) -> None:
        calls: list[int] = []
        timer = self.manager.create(0.1, lambda: calls.append(1))
        self.advance_to(0.1)
        timer.cancel()
        timer.cancel()  # 여러 번 불러도 안전
        self.advance_to(1.0)
        self.assertEqual(len(calls), 1)
        self.assertTrue(timer.is_canceled)
        self.assertEqual(self.manager.timer_count(), 0)

    def test_period_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            self.manager.create(0.0, lambda: None)
        with self.assertRaises(ValueError):
            self.manager.create(-1.0, lambda: None)

    def test_timer_created_later_counts_from_its_own_creation_time(self) -> None:
        self.source.set(10.0)
        calls: list[int] = []
        self.manager.create(0.5, lambda: calls.append(1))
        self.advance_to(10.4)
        self.assertEqual(calls, [])
        self.advance_to(10.5)
        self.assertEqual(len(calls), 1)

    def test_shutdown_stops_everything_and_rejects_new_timers(self) -> None:
        calls: list[int] = []
        self.manager.create(0.1, lambda: calls.append(1))
        self.manager.shutdown()
        self.manager.shutdown()  # 여러 번 불러도 안전
        self.source.set(5.0)
        self.assertEqual(calls, [])
        with self.assertRaises(RuntimeError):
            self.manager.create(0.1, lambda: None)

    def test_join_waits_for_a_running_callback(self) -> None:
        release = threading.Event()

        def wait_for_release() -> None:
            release.wait(5)

        self.manager.create(0.1, wait_for_release)
        self.source.set(0.1)
        self.assertFalse(self.manager.join(0.05))  # 아직 실행 중
        release.set()
        self.assertTrue(self.manager.join(5))


class TestTimerWithRealTime(unittest.TestCase):
    """실제 시간: 시간 오차가 있어서 느슨하게만 본다."""

    def test_fires_repeatedly_and_stops_after_cancel(self) -> None:
        manager = TimerManager()
        self.addCleanup(manager.shutdown)
        calls: list[float] = []
        timer = manager.create(0.05, lambda: calls.append(time.monotonic()))
        time.sleep(0.4)
        timer.cancel()
        count = len(calls)
        self.assertTrue(3 <= count <= 9, count)
        time.sleep(0.15)
        self.assertEqual(len(calls), count)  # 취소 뒤에는 멈춘다
        gaps = [b - a for a, b in zip(calls, calls[1:])]
        self.assertGreater(min(gaps), 0.03)  # 주기를 대체로 지킨다

    def test_scheduler_thread_stops_on_shutdown_and_is_lazy(self) -> None:
        def scheduler_threads() -> list[threading.Thread]:
            return [t for t in threading.enumerate() if t.name == "timer-scheduler"]

        before = len(scheduler_threads())
        manager = TimerManager()
        self.assertEqual(len(scheduler_threads()), before)  # 타이머를 만들기 전에는 스레드가 없다
        manager.create(10.0, lambda: None)
        self.assertEqual(len(scheduler_threads()), before + 1)
        manager.shutdown()
        self.assertEqual(len(scheduler_threads()), before)


class TestNodeAndContext(unittest.TestCase):
    def test_node_now_and_timer_use_the_kros_time_source(self) -> None:
        source = SimulatedTimeSource(100.0)
        kros = Kros(lambda ctx: [Node(ctx, "n")], time_source=source)
        self.addCleanup(kros.shutdown)
        node = kros.nodes[0]
        self.assertEqual(node.now(), 100.0)
        calls: list[float] = []
        timer = node.create_timer(0.5, lambda: calls.append(node.now()))
        source.set(100.5)
        kros.context.timer_manager.join()
        self.assertEqual(calls, [100.5])  # 콜백 안에서 본 시각도 시뮬레이션 시간
        node.destroy_timer(timer)
        source.set(102.0)
        kros.context.timer_manager.join()
        self.assertEqual(len(calls), 1)

    def test_default_time_source_is_the_system_clock(self) -> None:
        ctx = Context()
        self.addCleanup(ctx.shutdown)
        self.assertAlmostEqual(Node(ctx).now(), time.time(), delta=0.5)

    def test_context_shutdown_stops_timers_before_services_and_topics(self) -> None:
        source = SimulatedTimeSource()
        ctx = Context(time_source=source)
        node = Node(ctx)
        seen: list[int] = []
        node.subscribe("t", int, seen.append)
        node.create_timer(0.1, lambda: node.publish("t", 1))
        source.set(0.1)
        ctx.timer_manager.join()
        ctx.topic_manager.join()
        self.assertEqual(seen, [1])
        ctx.shutdown()
        source.set(1.0)  # 종료 뒤에는 타이머가 돌지 않는다
        self.assertEqual(seen, [1])

    def test_timer_callback_can_publish_and_call_a_service(self) -> None:
        from kros.service import ServiceType

        source = SimulatedTimeSource()
        ctx = Context(time_source=source)
        self.addCleanup(ctx.shutdown)
        node = Node(ctx)
        Echo = ServiceType("Echo", int, int)
        node.create_service("double", Echo, lambda x: x * 2)
        results: list[int] = []
        node.create_timer(0.1, lambda: results.append(node.call_service("double", Echo, 21, timeout=5)))
        source.set(0.1)
        ctx.timer_manager.join()
        self.assertEqual(results, [42])


if __name__ == "__main__":
    logging.basicConfig()
    unittest.main()
