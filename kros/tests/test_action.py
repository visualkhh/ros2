import threading
import time
import unittest
from collections.abc import Callable
from dataclasses import dataclass

from kros.action import ActionType, GoalStatus, ServerGoalHandle
from kros.context import Context
from kros.node import Node


@dataclass(frozen=True)
class CountGoal:
    n: int


@dataclass(frozen=True)
class CountFeedback:
    i: int


@dataclass(frozen=True)
class CountResult:
    total: int


Count = ActionType("Count", CountGoal, CountFeedback, CountResult)
H = ServerGoalHandle[CountGoal, CountFeedback]


def wait(event: threading.Event) -> None:
    assert event.wait(5), "timeout"


class TestAction(unittest.TestCase):
    def setUp(self) -> None:
        self.ctx = Context()
        self.addCleanup(self.ctx.shutdown)
        self.server_node = Node(self.ctx, "server")
        self.client_node = Node(self.ctx, "client")

    def counting(self, h: H) -> CountResult:
        for i in range(1, h.goal.n + 1):
            h.publish_feedback(CountFeedback(i))
        return CountResult(h.goal.n)

    def test_success_with_feedback(self) -> None:
        self.server_node.create_action_server("count", Count, self.counting)
        client = self.client_node.create_action_client("count", Count)
        got: list[CountFeedback] = []
        handle = client.send_goal(CountGoal(3), got.append)
        result = handle.result(5)
        self.ctx.topic_manager.join()
        self.assertEqual((result.status, result.result), (GoalStatus.SUCCEEDED, CountResult(3)))
        self.assertEqual(got, [CountFeedback(1), CountFeedback(2), CountFeedback(3)])
        self.assertEqual(handle.status, GoalStatus.SUCCEEDED)

    def test_cancel_running_goal(self) -> None:
        started = threading.Event()

        def execute(h: H) -> CountResult:
            started.set()
            for _ in range(500):
                if h.is_cancel_requested:
                    return CountResult(-1)
                time.sleep(0.01)
            return CountResult(0)

        self.server_node.create_action_server("count", Count, execute)
        client = self.client_node.create_action_client("count", Count)
        handle = client.send_goal(CountGoal(1))
        wait(started)
        self.assertTrue(handle.cancel(5))
        result = handle.result(5)
        self.assertEqual((result.status, result.result), (GoalStatus.CANCELED, CountResult(-1)))
        self.assertFalse(handle.cancel(5))  # 이미 끝남

    def test_goal_rejected_by_callback(self) -> None:
        self.server_node.create_action_server("count", Count, self.counting, goal_callback=lambda g: g.n > 0)
        client = self.client_node.create_action_client("count", Count)
        handle = client.send_goal(CountGoal(0))
        self.assertFalse(handle.accepted)
        self.assertEqual(handle.result(5).status, GoalStatus.REJECTED)

    def test_execute_exception_aborts(self) -> None:
        self.server_node.create_action_server("count", Count, lambda h: 1 / 0)  # type: ignore[arg-type,return-value]
        client = self.client_node.create_action_client("count", Count)
        with self.assertLogs("kros.action.action_server", "ERROR"):
            result = client.send_goal(CountGoal(1)).result(5)
        self.assertEqual((result.status, result.result), (GoalStatus.ABORTED, None))

    def test_wrong_result_type_aborts(self) -> None:
        self.server_node.create_action_server("count", Count, lambda h: "oops")  # type: ignore[arg-type,return-value]
        client = self.client_node.create_action_client("count", Count)
        with self.assertLogs("kros.action.action_server", "ERROR"):
            self.assertEqual(client.send_goal(CountGoal(1)).result(5).status, GoalStatus.ABORTED)

    def test_goals_run_one_at_a_time_in_order(self) -> None:
        order: list[int] = []

        def execute(h: H) -> CountResult:
            order.append(h.goal.n)
            return CountResult(h.goal.n)

        self.server_node.create_action_server("count", Count, execute)
        client = self.client_node.create_action_client("count", Count)
        handles = [client.send_goal(CountGoal(i)) for i in range(20)]
        for h in handles:
            h.result(5)
        self.assertEqual(order, list(range(20)))

    def test_cancel_queued_goal_never_runs(self) -> None:
        started, release = threading.Event(), threading.Event()
        ran: list[int] = []

        def execute(h: H) -> CountResult:
            ran.append(h.goal.n)
            started.set()
            wait(release)
            return CountResult(h.goal.n)

        self.server_node.create_action_server("count", Count, execute)
        client = self.client_node.create_action_client("count", Count)
        first = client.send_goal(CountGoal(1))
        wait(started)
        second = client.send_goal(CountGoal(2))  # 첫 goal이 실행 중이라 대기열
        self.assertTrue(second.cancel(5))
        release.set()
        self.assertEqual(first.result(5).status, GoalStatus.SUCCEEDED)
        self.assertEqual(second.result(5).status, GoalStatus.CANCELED)
        self.assertEqual(ran, [1])

    def test_wrong_goal_type_raises_in_client(self) -> None:
        self.server_node.create_action_server("count", Count, self.counting)
        client = self.client_node.create_action_client("count", Count)
        with self.assertRaises(TypeError):
            client.send_goal("nope")  # type: ignore[arg-type]

    def test_no_server_raises_lookup_error(self) -> None:
        client = self.client_node.create_action_client("count", Count)
        with self.assertRaises(LookupError):
            client.send_goal(CountGoal(1))

    def test_duplicate_server_raises(self) -> None:
        self.server_node.create_action_server("count", Count, self.counting)
        with self.assertRaises(ValueError):
            self.server_node.create_action_server("count", Count, self.counting)

    def test_two_clients_only_get_their_own_result(self) -> None:
        self.server_node.create_action_server("count", Count, self.counting)
        a = self.client_node.create_action_client("count", Count)
        b = Node(self.ctx, "client2").create_action_client("count", Count)
        fa: list[CountFeedback] = []
        fb: list[CountFeedback] = []
        ha, hb = a.send_goal(CountGoal(2), fa.append), b.send_goal(CountGoal(3), fb.append)
        ha.result(5), hb.result(5)
        self.ctx.topic_manager.join()
        self.assertEqual((len(fa), len(fb)), (2, 3))


class TestActionManager(unittest.TestCase):
    """TopicManager/ServiceManager와 같은 결: Context가 액션 매니저를 가지고, Node는 매니저를 거친다."""

    def setUp(self) -> None:
        self.ctx = Context()
        self.addCleanup(self.ctx.shutdown)
        self.server_node = Node(self.ctx, "server")
        self.client_node = Node(self.ctx, "client")
        self.manager = self.ctx.action_manager

    def blocking_execute(self, started: threading.Event, release: threading.Event) -> Callable[[H], CountResult]:
        def execute(h: H) -> CountResult:
            started.set()
            for _ in range(500):  # 취소 요청이 오면 일찍 끝낸다
                if h.is_cancel_requested or release.is_set():
                    return CountResult(-1)
                time.sleep(0.01)
            return CountResult(0)

        return execute

    def test_context_owns_the_action_manager_and_node_goes_through_it(self) -> None:
        self.assertEqual(self.manager.server_names(), [])
        server = self.server_node.create_action_server("count", Count, lambda h: CountResult(h.goal.n))
        self.assertEqual(self.manager.server_names(), ["count"])
        self.server_node.destroy_action_server(server)
        self.assertEqual(self.manager.server_names(), [])

    def test_duplicate_server_name_raises_value_error(self) -> None:
        self.server_node.create_action_server("count", Count, lambda h: CountResult(1))
        with self.assertRaises(ValueError):
            Node(self.ctx, "other").create_action_server("count", Count, lambda h: CountResult(2))

    def test_server_name_can_be_reused_after_destroy(self) -> None:
        server = self.server_node.create_action_server("count", Count, lambda h: CountResult(1))
        self.server_node.destroy_action_server(server)
        self.server_node.create_action_server("count", Count, lambda h: CountResult(h.goal.n))
        client = self.client_node.create_action_client("count", Count)
        self.assertEqual(client.send_goal(CountGoal(4)).result(5).result, CountResult(4))

    def test_destroyed_server_no_longer_accepts_goals(self) -> None:
        server = self.server_node.create_action_server("count", Count, lambda h: CountResult(1))
        client = self.client_node.create_action_client("count", Count)
        self.server_node.destroy_action_server(server)
        with self.assertRaises(LookupError):
            client.send_goal(CountGoal(1))

    def test_destroy_server_cancels_the_running_goal_and_releases_queued_ones(self) -> None:
        started, release = threading.Event(), threading.Event()
        server = self.server_node.create_action_server("count", Count, self.blocking_execute(started, release))
        client = self.client_node.create_action_client("count", Count)
        running = client.send_goal(CountGoal(1))
        wait(started)
        queued = client.send_goal(CountGoal(2))  # 첫 goal이 실행 중이라 대기열
        self.server_node.destroy_action_server(server)
        self.assertEqual(queued.result(5).status, GoalStatus.CANCELED)  # 시작도 못 한 goal: 영원히 기다리지 않는다
        self.assertEqual(running.result(5).status, GoalStatus.CANCELED)  # 실행 중이던 goal은 취소 요청을 받고 끝난다
        release.set()

    def test_destroy_client_releases_pending_handles(self) -> None:
        started, release = threading.Event(), threading.Event()
        self.server_node.create_action_server("count", Count, self.blocking_execute(started, release))
        client = self.client_node.create_action_client("count", Count)
        handle = client.send_goal(CountGoal(1))
        wait(started)
        self.client_node.destroy_action_client(client)
        self.assertEqual(handle.result(5).status, GoalStatus.ABORTED)  # 결과를 기다리던 핸들이 풀린다
        release.set()

    def test_destroy_client_twice_is_safe(self) -> None:
        client = self.client_node.create_action_client("count", Count)
        self.client_node.destroy_action_client(client)
        client.destroy()

    def test_context_shutdown_releases_everyone_waiting(self) -> None:
        started, release = threading.Event(), threading.Event()
        self.server_node.create_action_server("count", Count, self.blocking_execute(started, release))
        client = self.client_node.create_action_client("count", Count)
        running = client.send_goal(CountGoal(1))
        wait(started)
        queued = client.send_goal(CountGoal(2))
        self.ctx.shutdown()  # 액션 -> 서비스 -> 토픽 순서로 정리
        self.assertEqual(running.result(5).status, GoalStatus.ABORTED)
        self.assertEqual(queued.result(5).status, GoalStatus.ABORTED)
        self.assertEqual(self.manager.server_names(), [])
        release.set()

    def test_shutdown_is_idempotent(self) -> None:
        self.server_node.create_action_server("count", Count, lambda h: CountResult(1))
        self.ctx.shutdown()
        self.ctx.shutdown()


if __name__ == "__main__":
    unittest.main()
