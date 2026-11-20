import dataclasses
import unittest

from kros import Kros
from kros.context import Context
from kros.diagnostics import DIAGNOSTICS_TOPIC_NAME, DiagnosticStatusWrapper, DiagnosticUpdater
from kros.examples.turtlebot3.hardware_status import hardware_path, leaves, make_task
from kros.examples.turtlebot3.robot_bridge import RobotBridge
from kros.msgs.diagnostic_msgs import DiagnosticArray, DiagnosticLevel, DiagnosticStatus, KeyValue
from kros.msgs.std_msgs import Header
from kros.nodes import Node
from kros.nodes import DiagnosticAggregator
from kros.timer import SimulatedTimeSource
from kros.worlds.hardwares import Sensor
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3


class Watcher(Node):
    """diagnostics 토픽에 나온 것을 모은다."""

    def __init__(self, context: Context) -> None:
        super().__init__(context)
        self.arrays: list[DiagnosticArray] = []
        self.subscribe(DIAGNOSTICS_TOPIC_NAME, DiagnosticArray, self.arrays.append)


def settle(kros: Kros) -> None:
    kros.context.timer_manager.join()
    kros.context.topic_manager.join()


class TestMessages(unittest.TestCase):
    def test_levels_are_ordered_from_good_to_bad(self) -> None:
        self.assertTrue(DiagnosticLevel.OK < DiagnosticLevel.WARN < DiagnosticLevel.ERROR < DiagnosticLevel.STALE)
        self.assertEqual(max(DiagnosticLevel.WARN, DiagnosticLevel.ERROR), DiagnosticLevel.ERROR)

    def test_messages_are_immutable_plain_data(self) -> None:
        status = DiagnosticStatus(DiagnosticLevel.OK, "n", "m", "hw", (KeyValue("k", "v"),))
        array = DiagnosticArray(Header(1.5), (status,))
        with self.assertRaises(dataclasses.FrozenInstanceError):
            status.message = "x"  # type: ignore[misc]
        self.assertEqual(array.header.stamp, 1.5)

    def test_status_wrapper_builds_a_status(self) -> None:
        wrapper = DiagnosticStatusWrapper()
        self.assertEqual((wrapper.level, wrapper.message), (DiagnosticLevel.OK, ""))  # 안 적으면 OK
        wrapper.summary(DiagnosticLevel.WARN, "느리다")
        wrapper.add("rate", 4.5)
        status = wrapper.build("A: b", "LDS-01")
        self.assertEqual(status, DiagnosticStatus(DiagnosticLevel.WARN, "A: b", "느리다", "LDS-01", (KeyValue("rate", "4.5"),)))


class TestUpdater(unittest.TestCase):
    def setUp(self) -> None:
        self.source = SimulatedTimeSource()
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["watcher"], made["driver"] = Watcher(ctx), Node(ctx, "Driver")
            return list(made.values())

        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        watcher = made["watcher"]
        assert isinstance(watcher, Watcher)
        self.watcher: Watcher = watcher
        self.driver: Node = made["driver"]

    def advance(self, t: float) -> None:
        self.source.set(t)
        settle(self.kros)

    def test_publishes_every_period_with_the_kros_time_as_stamp(self) -> None:
        updater = DiagnosticUpdater(self.driver, period=1.0)
        updater.add("motor", lambda status: status.summary(DiagnosticLevel.OK, "ok"))
        self.advance(0.9)
        self.assertEqual(self.watcher.arrays, [])  # 첫 발행은 period 뒤
        self.advance(1.0)
        self.advance(2.0)
        self.assertEqual([a.header.stamp for a in self.watcher.arrays], [1.0, 2.0])

    def test_status_name_is_node_and_task_and_hardware_id_is_attached(self) -> None:
        updater = DiagnosticUpdater(self.driver, period=1.0, hardware_id="XL430")
        updater.add("motor", lambda status: status.summary(DiagnosticLevel.WARN, "뜨겁다"))
        self.advance(1.0)
        status = self.watcher.arrays[0].status[0]
        self.assertEqual((status.name, status.hardware_id, status.level, status.message), ("Driver: motor", "XL430", DiagnosticLevel.WARN, "뜨겁다"))

    def test_all_tasks_go_out_in_one_array(self) -> None:
        updater = DiagnosticUpdater(self.driver, period=1.0)
        updater.add("a", lambda status: status.add("x", 1))
        updater.add("b", lambda status: status.add("y", 2))
        self.advance(1.0)
        self.assertEqual([s.name for s in self.watcher.arrays[0].status], ["Driver: a", "Driver: b"])
        self.assertEqual(self.watcher.arrays[0].status[1].values, (KeyValue("y", "2"),))

    def test_a_failing_task_becomes_error_and_the_others_still_go_out(self) -> None:
        updater = DiagnosticUpdater(self.driver, period=1.0)

        def broken(status: DiagnosticStatusWrapper) -> None:
            status.add("half", "written")
            raise RuntimeError("센서가 응답하지 않는다")

        updater.add("broken", broken)
        updater.add("fine", lambda status: status.summary(DiagnosticLevel.OK, "ok"))
        with self.assertLogs("kros.diagnostics.diagnostic_updater", level="ERROR"):
            self.advance(1.0)
        bad, good = self.watcher.arrays[0].status
        self.assertEqual(bad.level, DiagnosticLevel.ERROR)
        self.assertIn("센서가 응답하지 않는다", bad.message)
        self.assertEqual(bad.values, ())  # 실패한 작업이 반쯤 쓴 값은 내보내지 않는다
        self.assertEqual(good.level, DiagnosticLevel.OK)

    def test_no_tasks_publishes_nothing(self) -> None:
        DiagnosticUpdater(self.driver, period=1.0)
        self.advance(3.0)
        self.assertEqual(self.watcher.arrays, [])

    def test_duplicate_task_name_is_rejected_and_remove_works(self) -> None:
        updater = DiagnosticUpdater(self.driver, period=1.0)
        updater.add("a", lambda status: None)
        with self.assertRaises(ValueError):
            updater.add("a", lambda status: None)
        updater.remove("a")
        updater.add("a", lambda status: None)  # 지운 뒤에는 다시 쓸 수 있다

    def test_set_period_changes_the_rate_and_bad_period_keeps_the_old_one(self) -> None:
        updater = DiagnosticUpdater(self.driver, period=1.0)
        updater.add("a", lambda status: None)
        self.advance(1.0)
        updater.set_period(0.5)
        self.advance(1.5)
        self.assertEqual(len(self.watcher.arrays), 2)  # 바꾼 시각(1.0)부터 0.5초 뒤인 1.5에 발행
        with self.assertRaises(ValueError):
            updater.set_period(0.0)
        self.assertEqual(updater.period, 0.5)
        self.advance(2.0)
        self.assertEqual(len(self.watcher.arrays), 3)  # 예전 주기(0.5)로 계속 돈다

    def test_close_stops_publishing(self) -> None:
        updater = DiagnosticUpdater(self.driver, period=1.0)
        updater.add("a", lambda status: None)
        self.advance(1.0)
        updater.close()
        updater.close()
        self.advance(5.0)
        self.assertEqual(len(self.watcher.arrays), 1)

    def test_manual_update_returns_and_publishes_the_array(self) -> None:
        updater = DiagnosticUpdater(self.driver, period=10.0)
        updater.add("a", lambda status: status.summary(DiagnosticLevel.OK, "x"))
        array = updater.update()
        settle(self.kros)
        self.assertIsNotNone(array)
        self.assertEqual(len(self.watcher.arrays), 1)


class TestAggregator(unittest.TestCase):
    def setUp(self) -> None:
        self.source = SimulatedTimeSource()
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["aggregator"], made["publisher"] = DiagnosticAggregator(ctx), Node(ctx, "Pub")
            return list(made.values())

        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        aggregator = made["aggregator"]
        assert isinstance(aggregator, DiagnosticAggregator)
        self.aggregator: DiagnosticAggregator = aggregator
        self.publisher: Node = made["publisher"]

    def publish(self, name: str, level: DiagnosticLevel, message: str = "") -> None:
        self.publisher.publish(DIAGNOSTICS_TOPIC_NAME, DiagnosticArray(Header(self.source.now()), (DiagnosticStatus(level, name, message),)))
        self.kros.context.topic_manager.join()

    def test_collects_the_latest_status_per_name(self) -> None:
        self.publish("a", DiagnosticLevel.OK)
        self.publish("a", DiagnosticLevel.WARN, "느리다")
        self.publish("b", DiagnosticLevel.OK)
        statuses = self.aggregator.statuses()
        self.assertEqual({n: s.level for n, s in statuses.items()}, {"a": DiagnosticLevel.WARN, "b": DiagnosticLevel.OK})

    def test_silent_names_become_stale_after_the_timeout(self) -> None:
        self.publish("a", DiagnosticLevel.OK, "ok")
        self.source.set(4.9)
        self.assertEqual(self.aggregator.statuses()["a"].level, DiagnosticLevel.OK)
        self.source.set(5.1)  # stale_timeout 5초 넘게 소식이 없다
        stale = self.aggregator.statuses()["a"]
        self.assertEqual(stale.level, DiagnosticLevel.STALE)
        self.assertIn("ok", stale.message)  # 마지막으로 알려 준 내용이 남는다

    def test_a_new_report_clears_stale(self) -> None:
        self.publish("a", DiagnosticLevel.OK)
        self.source.set(10.0)
        self.assertEqual(self.aggregator.statuses()["a"].level, DiagnosticLevel.STALE)
        self.publish("a", DiagnosticLevel.OK)
        self.assertEqual(self.aggregator.statuses()["a"].level, DiagnosticLevel.OK)

    def test_stale_timeout_is_a_parameter_that_can_change_while_running(self) -> None:
        self.publish("a", DiagnosticLevel.OK)
        self.source.set(3.0)
        self.assertEqual(self.aggregator.statuses()["a"].level, DiagnosticLevel.OK)
        self.kros.set_parameter(self.aggregator, "stale_timeout", 2.0)
        self.assertEqual(self.aggregator.statuses()["a"].level, DiagnosticLevel.STALE)
        with self.assertRaises(Exception):
            self.kros.set_parameter(self.aggregator, "stale_timeout", -1.0)

    def test_worst_level_and_nothing_heard_yet_is_stale_not_ok(self) -> None:
        self.assertEqual(self.aggregator.worst_level(), DiagnosticLevel.STALE)  # 아직 아무 소식도 없다: 모르는 것이지 정상이 아니다
        self.publish("a", DiagnosticLevel.OK)
        self.assertEqual(self.aggregator.worst_level(), DiagnosticLevel.OK)
        self.publish("b", DiagnosticLevel.ERROR)
        self.publish("c", DiagnosticLevel.WARN)
        self.assertEqual(self.aggregator.worst_level(), DiagnosticLevel.ERROR)


class TestHardwareDiagnostics(unittest.TestCase):
    """하드웨어가 남긴 사실이 진단 수준으로 바뀐다 (판단은 kros 쪽 접착제가 한다)."""

    def setUp(self) -> None:
        self.source = SimulatedTimeSource()
        self.world, self.robot = MazeWorld(puck_count=0), TurtleBot3()
        self.world.add(self.robot)
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["watcher"], made["aggregator"], made["bridge"] = Watcher(ctx), DiagnosticAggregator(ctx), RobotBridge(ctx, self.robot)
            return list(made.values())

        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        watcher, aggregator = made["watcher"], made["aggregator"]
        assert isinstance(watcher, Watcher) and isinstance(aggregator, DiagnosticAggregator)
        self.watcher: Watcher = watcher
        self.aggregator: DiagnosticAggregator = aggregator

    def run_world(self, seconds: float) -> None:
        for _ in range(round(seconds / 0.05)):
            self.world.step(0.05)
            self.source.set(self.world.time)  # 시뮬레이션이 kros 시간을 밀어 준다
            self.kros.context.timer_manager.join()  # 시간을 sleep 없이 몰아서 밀면 타이머 워커가 못 따라가 주기를 건너뛴다 (missed). 한 스텝마다 기다린다
        settle(self.kros)

    def latest(self) -> dict[str, DiagnosticStatus]:
        return {s.name.split(": ", 1)[1]: s for s in self.watcher.arrays[-1].status}

    def test_hardware_tree_becomes_named_tasks(self) -> None:
        names = {hardware_path(h) for h in leaves(self.robot)}
        self.assertEqual(
            names,
            {
                "turtlebot3_waffle_pi/drive",
                "turtlebot3_waffle_pi/drive/left_wheel",
                "turtlebot3_waffle_pi/drive/right_wheel",
                "turtlebot3_waffle_pi/lidar",
            },
        )

    def test_healthy_robot_reports_ok_with_hardware_ids(self) -> None:
        self.run_world(2.0)
        statuses = self.latest()
        self.assertEqual({s.level for s in statuses.values()}, {DiagnosticLevel.OK})
        self.assertEqual(statuses["turtlebot3_waffle_pi/lidar"].hardware_id, "turtlebot3_waffle_pi")
        values = dict((kv.key, kv.value) for kv in statuses["turtlebot3_waffle_pi/lidar"].values)
        self.assertEqual((values["model"], values["powered"], values["error_count"]), ("LDS-01", "True", "0"))

    def test_new_read_failures_warn_once_then_clear(self) -> None:
        self.run_world(2.0)
        self.robot.lidar.fault = lambda clock: 2.0 < clock.time <= 2.45  # 잠깐 통신이 끊겼다
        self.run_world(1.0)
        warns = [a.status for a in self.watcher.arrays if any(s.level is DiagnosticLevel.WARN for s in a.status)]
        self.assertTrue(warns, "실패가 진단에 나타나지 않았다")
        lidar = next(s for s in warns[0] if s.name.endswith("lidar"))
        self.assertIn("새 실패", lidar.message)
        self.run_world(3.0)  # 회복한 뒤에는 다시 OK (실패 기록은 누적으로 남지만 새 실패는 아니다)
        self.assertEqual(self.latest()["turtlebot3_waffle_pi/lidar"].level, DiagnosticLevel.OK)

    def test_sensor_with_no_new_data_for_too_long_is_an_error(self) -> None:
        self.run_world(2.0)
        self.robot.lidar.fault = lambda clock: True  # 완전히 끊겼다
        self.run_world(4.0)  # data_timeout 2초보다 오래
        lidar = self.latest()["turtlebot3_waffle_pi/lidar"]
        self.assertEqual(lidar.level, DiagnosticLevel.ERROR)
        self.assertIn("새 값이 없다", lidar.message)  # 가장 나쁜 문제가 설명의 맨 앞에 온다
        self.assertTrue(lidar.message.startswith("2") or lidar.message.startswith("3") or lidar.message.startswith("4"), lidar.message)

    def test_powered_off_hardware_warns_and_parents_power_applies_to_parts(self) -> None:
        self.run_world(2.0)
        self.robot.drive.power_off()  # 구동부를 끄면 바퀴 모터도 꺼진다
        self.run_world(1.5)
        statuses = self.latest()
        for name in ("drive", "drive/left_wheel", "drive/right_wheel"):
            self.assertEqual(statuses[f"turtlebot3_waffle_pi/{name}"].level, DiagnosticLevel.WARN, name)
        self.assertEqual(statuses["turtlebot3_waffle_pi/lidar"].level, DiagnosticLevel.OK)

    def test_sensor_that_never_produced_warns(self) -> None:
        fresh = TurtleBot3().lidar  # 월드에 놓지 않아 한 번도 값을 못 냈다
        self.assertIsInstance(fresh, Sensor)
        wrapper = DiagnosticStatusWrapper()
        make_task(fresh, lambda: 0.0, lambda: 2.0)(wrapper)
        self.assertEqual((wrapper.level, wrapper.message), (DiagnosticLevel.WARN, "아직 값이 없다"))

    def test_several_problems_are_all_reported_with_the_worst_first(self) -> None:
        self.run_world(2.0)
        self.robot.lidar.fault = lambda clock: True
        self.robot.lidar.power_off()
        self.run_world(1.5)  # 진단은 1초마다 나가니 그 주기를 넘겨서 돌려야 끈 뒤의 진단이 나온다
        lidar = self.latest()["turtlebot3_waffle_pi/lidar"]
        self.assertEqual(lidar.level, DiagnosticLevel.WARN)  # 꺼져 있으면 읽기를 시도하지 않아 새 실패는 없고 전원만 문제다
        self.assertIn("전원이 꺼져 있다", lidar.message)

    def test_diagnostics_period_and_timeout_are_live_parameters(self) -> None:
        self.run_world(2.0)
        before = len(self.watcher.arrays)
        self.kros.set_parameter("RobotBridge", "diagnostics_period", 0.25)  # 더 자주
        self.run_world(1.0)
        self.assertGreaterEqual(len(self.watcher.arrays) - before, 3)
        self.robot.lidar.fault = lambda clock: True
        self.kros.set_parameter("RobotBridge", "data_timeout", 0.5)  # 더 엄격하게
        self.run_world(1.5)
        self.assertEqual(self.latest()["turtlebot3_waffle_pi/lidar"].level, DiagnosticLevel.ERROR)

    def test_aggregator_sees_the_robot_and_goes_stale_when_the_robot_stops_reporting(self) -> None:
        self.run_world(2.0)
        self.assertEqual(self.aggregator.worst_level(), DiagnosticLevel.OK)
        self.kros.context.timer_manager.shutdown()  # 로봇 쪽 진단 타이머가 멈췄다 (로봇이 소식을 안 보낸다)
        self.source.set(self.source.now() + 6.0)  # 시간은 stale_timeout(5초)보다 더 흐른다
        self.assertEqual(self.aggregator.worst_level(), DiagnosticLevel.STALE)  # 소식이 끊기면 정상이 아니라 모르는 것으로 본다


if __name__ == "__main__":
    unittest.main()
