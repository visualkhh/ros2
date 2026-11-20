import logging
import threading
import unittest

from kros import Kros
from kros.context import Context
from kros.log import LOG_TOPIC_NAME, enable_console
from kros.msgs.rcl_interfaces import Log, LogLevel
from kros.node import Node
from kros.parameter import ParameterRejectedError
from kros.timer import SimulatedTimeSource


class Listener(Node):
    """rosout 토픽에 나온 로그를 모은다."""

    def __init__(self, context: Context) -> None:
        super().__init__(context)
        self.logs: list[Log] = []
        self.subscribe(LOG_TOPIC_NAME, Log, self.logs.append)


class Worker(Node):
    pass


def make(parameters: dict[str, dict[str, bool | int | float | str]] | None = None) -> tuple[Kros, Listener, Worker, SimulatedTimeSource]:
    source = SimulatedTimeSource(10.0)
    made: dict[str, Node] = {}

    def create(ctx: Context) -> list[Node]:
        made["listener"], made["worker"] = Listener(ctx), Worker(ctx)
        return list(made.values())

    kros = Kros(create, parameters=parameters, time_source=source)
    listener, worker = made["listener"], made["worker"]
    assert isinstance(listener, Listener) and isinstance(worker, Worker)
    return kros, listener, worker, source


def settle(kros: Kros) -> None:
    kros.context.topic_manager.join()


class TestLogMessage(unittest.TestCase):
    def test_levels_match_python_logging(self) -> None:
        self.assertEqual([int(level) for level in LogLevel], [10, 20, 30, 40, 50])
        self.assertEqual(LogLevel.WARN, logging.WARNING)
        self.assertEqual(LogLevel.FATAL, logging.CRITICAL)

    def test_log_is_immutable_plain_data(self) -> None:
        log = Log(1.0, LogLevel.INFO, "n", "m")
        self.assertEqual((log.file, log.function, log.line), ("", "", 0))


class TestRosout(unittest.TestCase):
    def setUp(self) -> None:
        self.kros, self.listener, self.worker, self.source = make()
        self.addCleanup(self.kros.shutdown)
        self.logger = self.worker.get_logger()

    def test_log_goes_to_the_rosout_topic_with_time_name_and_caller(self) -> None:
        self.source.set(12.5)
        self.logger.info("시작했다")
        settle(self.kros)
        (log,) = self.listener.logs
        self.assertEqual((log.stamp, log.level, log.name, log.msg), (12.5, LogLevel.INFO, "Worker", "시작했다"))  # 시각은 kros 시간
        self.assertTrue(log.file.endswith("test_log.py"))
        self.assertEqual(log.function, "test_log_goes_to_the_rosout_topic_with_time_name_and_caller")  # 부른 곳이 찍힌다
        self.assertGreater(log.line, 0)

    def test_all_levels_have_a_method(self) -> None:
        self.worker.set_parameter("log_level", "DEBUG")
        self.logger.debug("d")
        self.logger.info("i")
        self.logger.warn("w")
        self.logger.error("e")
        self.logger.fatal("f")
        settle(self.kros)
        self.assertEqual([log.level for log in self.listener.logs], [LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR, LogLevel.FATAL])

    def test_percent_arguments_like_python_logging(self) -> None:
        self.logger.info("속도 %.2f m/s, 이름 %s", 0.256, "wheel")
        settle(self.kros)
        self.assertEqual(self.listener.logs[0].msg, "속도 0.26 m/s, 이름 wheel")

    def test_bad_formatting_never_raises(self) -> None:
        self.logger.info("%d개", "여럿")  # 로그가 프로그램을 죽이면 안 된다
        self.logger.info("인자가 모자라다 %s %s", 1)
        settle(self.kros)
        self.assertEqual(len(self.listener.logs), 2)
        self.assertIn("포매팅 실패", self.listener.logs[0].msg)

    def test_percent_sign_without_arguments_is_left_alone(self) -> None:
        self.logger.info("배터리 50%")
        settle(self.kros)
        self.assertEqual(self.listener.logs[0].msg, "배터리 50%")

    def test_python_logging_gets_it_too_with_mapped_levels(self) -> None:
        self.worker.set_parameter("log_level", "DEBUG")
        with self.assertLogs("kros.node.Worker", level="DEBUG") as captured:
            self.logger.debug("d")
            self.logger.warn("w")
            self.logger.fatal("f")
        self.assertEqual([(r.levelno, r.getMessage()) for r in captured.records], [(logging.DEBUG, "d"), (logging.WARNING, "w"), (logging.CRITICAL, "f")])
        self.assertTrue(captured.records[0].pathname.endswith("test_log.py"))  # 파이썬 로그에도 부른 곳이 찍힌다


class TestLogLevelParameter(unittest.TestCase):
    def setUp(self) -> None:
        self.kros, self.listener, self.worker, self.source = make()
        self.addCleanup(self.kros.shutdown)
        self.logger = self.worker.get_logger()

    def test_every_node_declares_log_level_with_info_as_default(self) -> None:
        self.assertEqual(self.worker.get_parameter("log_level"), "INFO")
        self.assertEqual(self.listener.get_parameter("log_level"), "INFO")

    def test_below_the_level_nothing_goes_out(self) -> None:
        self.logger.debug("보이지 않는다")
        settle(self.kros)
        self.assertEqual(self.listener.logs, [])
        with self.assertNoLogs("kros.node.Worker", level="DEBUG"):
            self.logger.debug("보이지 않는다")

    def test_changing_the_level_while_running_takes_effect_immediately(self) -> None:
        self.logger.debug("전")
        self.kros.set_parameter(self.worker, "log_level", "DEBUG")
        self.logger.debug("후")
        self.kros.set_parameter(self.worker, "log_level", "ERROR")
        self.logger.warn("가려진다")
        self.logger.error("나온다")
        settle(self.kros)
        self.assertEqual([log.msg for log in self.listener.logs], ["후", "나온다"])

    def test_level_can_be_given_when_starting_kros(self) -> None:
        kros, listener, worker, _ = make({"Worker": {"log_level": "ERROR"}})
        self.addCleanup(kros.shutdown)
        worker.get_logger().info("가려진다")
        worker.get_logger().error("나온다")
        settle(kros)
        self.assertEqual([log.msg for log in listener.logs], ["나온다"])

    def test_level_name_is_case_insensitive_but_must_be_a_real_level(self) -> None:
        self.worker.set_parameter("log_level", "debug")
        self.logger.debug("소문자도 된다")
        settle(self.kros)
        self.assertEqual(len(self.listener.logs), 1)
        with self.assertRaises(ParameterRejectedError):
            self.worker.set_parameter("log_level", "LOUD")
        self.assertEqual(self.worker.get_parameter("log_level"), "debug")  # 거절되면 그대로

    def test_nodes_have_their_own_levels(self) -> None:
        self.worker.set_parameter("log_level", "DEBUG")
        self.listener.get_logger().debug("가려진다")  # Listener는 아직 INFO
        self.logger.debug("나온다")
        settle(self.kros)
        self.assertEqual([(log.name, log.msg) for log in self.listener.logs], [("Worker", "나온다")])

    def test_is_enabled_for_lets_callers_skip_expensive_messages(self) -> None:
        self.assertFalse(self.logger.is_enabled_for(LogLevel.DEBUG))
        self.assertTrue(self.logger.is_enabled_for(LogLevel.INFO))
        self.worker.set_parameter("log_level", "DEBUG")
        self.assertTrue(self.logger.is_enabled_for(LogLevel.DEBUG))

    def test_changing_the_level_is_announced_on_parameter_events(self) -> None:
        from kros.parameter import PARAMETER_EVENTS_TOPIC_NAME, ParameterEvent

        events: list[ParameterEvent] = []
        self.listener.subscribe(PARAMETER_EVENTS_TOPIC_NAME, ParameterEvent, events.append)
        self.kros.set_parameter(self.worker, "log_level", "DEBUG")
        settle(self.kros)
        self.assertEqual([(e.node, e.name, e.old, e.new) for e in events], [("Worker", "log_level", "INFO", "DEBUG")])


class TestThreads(unittest.TestCase):
    def test_logging_from_timer_subscriber_and_plain_threads(self) -> None:
        kros, listener, worker, source = make()
        self.addCleanup(kros.shutdown)
        logger = worker.get_logger()
        worker.create_timer(1.0, lambda: logger.info("타이머"))
        worker.subscribe("t", int, lambda v: logger.info("구독 %d", v))
        def spam(i: int) -> None:
            for j in range(50):
                logger.info("스레드 %d-%d", i, j)

        threads = [threading.Thread(target=spam, args=(i,)) for i in range(4)]
        for t in threads:
            t.start()
        worker.publish("t", 7)
        for t in threads:
            t.join()
        source.set(source.now() + 1.0)
        kros.context.timer_manager.join()
        settle(kros)
        messages = [log.msg for log in listener.logs]
        self.assertEqual(len(messages), 4 * 50 + 2)
        self.assertIn("타이머", messages)
        self.assertIn("구독 7", messages)


class TestConsole(unittest.TestCase):
    def test_enable_console_installs_one_handler_with_a_readable_format(self) -> None:
        console = logging.getLogger("kros.node")
        before = [h for h in console.handlers if h.get_name() == "kros-console"]
        for handler in before:
            console.removeHandler(handler)

        def remove_console_handlers() -> None:
            for handler in list(console.handlers):
                if handler.get_name() == "kros-console":
                    console.removeHandler(handler)

        self.addCleanup(remove_console_handlers)
        first = enable_console(logging.INFO)
        second = enable_console(logging.DEBUG)  # 여러 번 불러도 핸들러는 하나
        self.assertIs(first, second)
        self.assertEqual(len([h for h in console.handlers if h.get_name() == "kros-console"]), 1)
        self.assertEqual(second.level, logging.DEBUG)
        record = logging.LogRecord("kros.node.Worker", logging.WARNING, "f.py", 1, "배터리 낮음", None, None)
        self.assertEqual(second.format(record), "[WARNING] [kros.node.Worker]: 배터리 낮음")


if __name__ == "__main__":
    unittest.main()
