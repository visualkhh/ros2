import logging
import unittest

from kros import Kros
from kros.context import Context
from kros.nodes import Node
from kros.parameter import (
    PARAMETER_EVENTS_TOPIC_NAME,
    ParameterAlreadyDeclaredError,
    ParameterEvent,
    ParameterEventKind,
    ParameterManager,
    ParameterNotDeclaredError,
    ParameterRejectedError,
)


class Speedy(Node):
    def __init__(self, context: Context, name: str | None = None, parameters: dict[str, bool | int | float | str] | None = None) -> None:
        super().__init__(context, name, parameters)
        self.speed = self.declare_parameter("speed", 0.26)
        self.topic = self.declare_parameter("topic", "cmd_vel")
        self.count = self.declare_parameter("count", 3)


class TestNodeName(unittest.TestCase):
    def test_default_name_is_the_class_name(self) -> None:
        ctx = Context()
        self.assertEqual(Speedy(ctx).name, "Speedy")
        self.assertEqual(Node(ctx).name, "Node")

    def test_duplicate_names_are_rejected_unless_named_explicitly(self) -> None:
        ctx = Context()
        Speedy(ctx)
        with self.assertRaises(ValueError):
            Speedy(ctx)
        self.assertEqual(Speedy(ctx, "second").name, "second")  # 같은 클래스를 여럿 만들 때는 이름을 따로

    def test_names_are_per_context(self) -> None:
        Speedy(Context())
        Speedy(Context())  # 다른 Context: 겹치지 않는다


class TestDeclareAndGet(unittest.TestCase):
    def test_declared_default_and_live_value(self) -> None:
        node = Speedy(Context())
        self.assertEqual(node.speed.value, 0.26)
        node.speed.set(0.5)
        self.assertEqual(node.speed.value, 0.5)  # 손잡이는 항상 현재 값
        self.assertEqual(node.get_parameter("speed"), 0.5)
        node.set_parameter("speed", 0.1)
        self.assertEqual(node.speed.value, 0.1)

    def test_type_is_locked_by_the_default(self) -> None:
        node = Speedy(Context())
        with self.assertRaises(TypeError):
            node.speed.set("fast")  # type: ignore[arg-type]
        with self.assertRaises(TypeError):
            node.set_parameter("speed", 1)  # int는 float이 아니다 (타입은 정확히 같아야 한다)
        with self.assertRaises(TypeError):
            node.set_parameter("count", True)  # bool은 int가 아니다
        self.assertEqual((node.speed.value, node.count.value), (0.26, 3))  # 거절된 값은 반영되지 않는다

    def test_unsupported_types_are_rejected_at_declare(self) -> None:
        node = Node(Context())
        with self.assertRaises(TypeError):
            node.declare_parameter("x", {"a": 1})  # type: ignore[type-var]

    def test_tuple_parameters_keep_their_element_type(self) -> None:
        node = Node(Context())
        default: tuple[str, ...] = ("red", "blue")
        p = node.declare_parameter("colors", default)
        p.set(("green",))
        self.assertEqual(p.value, ("green",))
        with self.assertRaises(TypeError):
            p.set((1, 2))  # type: ignore[arg-type]

    def test_undeclared_and_double_declare(self) -> None:
        node = Speedy(Context())
        with self.assertRaises(ParameterNotDeclaredError):
            node.get_parameter("nope")
        with self.assertRaises(ParameterNotDeclaredError):
            node.set_parameter("nope", 1)
        with self.assertRaises(ParameterAlreadyDeclaredError):
            node.declare_parameter("speed", 1.0)

    def test_parameters_are_per_node(self) -> None:
        ctx = Context()
        a, b = Speedy(ctx, "a"), Speedy(ctx, "b")
        a.speed.set(9.0)
        self.assertEqual(b.speed.value, 0.26)


class TestRemoteAdoption(unittest.TestCase):
    def test_remote_snapshot_does_not_overwrite_local_or_rebroadcast(self) -> None:
        ctx = Context()
        pm = ctx.parameter_manager
        node = Node(ctx, "controller")
        speed = node.declare_parameter("speed", 0.5)
        pm.adopt("controller", "speed", 0.1)  # 재접속한 Viz가 보낸 옛 값
        self.assertEqual(speed.value, 0.5)  # 서버의 로컬 값은 그대로
        pm.adopt("viz", "zoom", 2.0)  # 원격 노드 값은 받아 둔다
        self.assertEqual(pm.get("viz", "zoom"), 2.0)
        declared = pm.all_declared()
        self.assertEqual(declared["controller"]["speed"], 0.5)
        self.assertNotIn("viz", declared)  # 받은 원격 값을 다시 방송하지 않는다 (핑퐁/되돌림 방지)
        pm.adopt("viz", "zoom", 3.0)  # 원격 값 갱신은 된다
        self.assertEqual(pm.get("viz", "zoom"), 3.0)


class TestValidationAndCallbacks(unittest.TestCase):
    def test_validator_can_reject_a_change(self) -> None:
        node = Node(Context())
        period = node.declare_parameter("period", 0.2, validator=lambda v: v > 0)
        with self.assertRaises(ParameterRejectedError):
            period.set(-1.0)
        self.assertEqual(period.value, 0.2)  # 거절되면 그대로
        period.set(0.5)
        self.assertEqual(period.value, 0.5)

    def test_on_change_is_called_after_a_successful_change_only(self) -> None:
        node = Node(Context())
        seen: list[float] = []
        period = node.declare_parameter("period", 0.2, validator=lambda v: v > 0, on_change=seen.append)
        period.set(0.4)
        with self.assertRaises(ParameterRejectedError):
            period.set(-1.0)
        self.assertEqual(seen, [0.4])

    def test_overrides_must_pass_the_validator_at_declare_time(self) -> None:
        ctx = Context({"Node": {"period": -5.0}})
        with self.assertRaises(ParameterRejectedError):
            Node(ctx).declare_parameter("period", 0.2, validator=lambda v: v > 0)


class TestPrecedence(unittest.TestCase):
    """선언한 기본값 < Node(parameters=...) < Kros(parameters=...) < 실행 중 set."""

    def test_default_then_node_then_kros_then_runtime(self) -> None:
        self.assertEqual(Speedy(Context()).speed.value, 0.26)  # 기본값
        self.assertEqual(Speedy(Context(), parameters={"speed": 0.3}).speed.value, 0.3)  # Node가 준 값
        ctx = Context({"Speedy": {"speed": 0.4}})
        self.assertEqual(Speedy(ctx, parameters={"speed": 0.3}).speed.value, 0.4)  # Kros가 준 값이 Node가 준 값보다 강하다
        node = Speedy(Context({"Speedy": {"speed": 0.4}}))
        node.speed.set(0.9)
        self.assertEqual(node.speed.value, 0.9)  # 실행 중 set이 가장 강하다

    def test_overrides_are_keyed_by_node_name(self) -> None:
        ctx = Context({"left": {"speed": 0.1}})
        left, right = Speedy(ctx, "left"), Speedy(ctx, "right")
        self.assertEqual((left.speed.value, right.speed.value), (0.1, 0.26))

    def test_override_with_the_wrong_type_fails_at_declare(self) -> None:
        with self.assertRaises(TypeError):
            Speedy(Context({"Speedy": {"speed": "fast"}}))


class TestDynamicChange(unittest.TestCase):
    """실행 중에 바꾸면 쓰는 쪽에도 바로 반영된다."""

    def test_kros_set_parameter_by_node_object_or_name(self) -> None:
        kros = Kros(lambda ctx: [Speedy(ctx)])
        self.addCleanup(kros.shutdown)
        node = kros.nodes[0]
        assert isinstance(node, Speedy)
        kros.set_parameter(node, "speed", 0.1)  # 노드 객체로
        self.assertEqual(node.speed.value, 0.1)
        kros.set_parameter("Speedy", "speed", 0.2)  # 이름으로
        self.assertEqual((node.speed.value, kros.get_parameter(node, "speed")), (0.2, 0.2))

    def test_kros_set_parameter_checks_type_and_validator(self) -> None:
        class Checked(Node):
            def __init__(self, context: Context) -> None:
                super().__init__(context)
                self.limit = self.declare_parameter("limit", 10, validator=lambda v: v >= 0)

        kros = Kros(lambda ctx: [Checked(ctx)])
        self.addCleanup(kros.shutdown)
        with self.assertRaises(TypeError):
            kros.set_parameter("Checked", "limit", "many")
        with self.assertRaises(ParameterRejectedError):
            kros.set_parameter("Checked", "limit", -1)
        with self.assertRaises(ParameterNotDeclaredError):
            kros.set_parameter("Checked", "nope", 1)
        self.assertEqual(kros.get_parameter("Checked", "limit"), 10)

    def test_code_that_reads_value_each_time_sees_the_change(self) -> None:
        class Driver(Node):
            def __init__(self, context: Context) -> None:
                super().__init__(context)
                self.speed = self.declare_parameter("speed", 1.0)

            def command(self) -> float:
                return self.speed.value * 2  # 쓸 때마다 value를 읽는다: 바뀐 값이 바로 반영된다

        kros = Kros(lambda ctx: [Driver(ctx)])
        self.addCleanup(kros.shutdown)
        node = kros.nodes[0]
        assert isinstance(node, Driver)
        self.assertEqual(node.command(), 2.0)
        kros.set_parameter(node, "speed", 3.0)
        self.assertEqual(node.command(), 6.0)

    def test_caching_value_in_a_variable_does_not_follow_changes(self) -> None:
        node = Speedy(Context())
        cached = node.speed.value  # 이렇게 값을 변수에 복사해 두면 따라 바뀌지 않는다 (규칙: 쓸 때마다 .value)
        node.speed.set(5.0)
        self.assertEqual((cached, node.speed.value), (0.26, 5.0))

    def test_on_change_lets_a_node_react_to_a_change(self) -> None:
        class Periodic(Node):
            def __init__(self, context: Context) -> None:
                super().__init__(context)
                self.ticks = 0
                self.period = self.declare_parameter("period", 0.2, validator=lambda v: v > 0, on_change=self._rebuild)

            def _rebuild(self, period: float) -> None:
                self.ticks += 1  # 값에서 파생된 상태를 다시 만들어야 할 때

        kros = Kros(lambda ctx: [Periodic(ctx)])
        self.addCleanup(kros.shutdown)
        kros.set_parameter("Periodic", "period", 0.5)
        with self.assertRaises(ParameterRejectedError):
            kros.set_parameter("Periodic", "period", 0.0)
        node = kros.nodes[0]
        assert isinstance(node, Periodic)
        self.assertEqual((node.ticks, node.period.value), (1, 0.5))

    def test_change_from_another_thread_is_seen_by_a_running_subscriber(self) -> None:
        import threading

        class Scaler(Node):
            def __init__(self, context: Context) -> None:
                super().__init__(context)
                self.gain = self.declare_parameter("gain", 1)
                self.out: list[int] = []
                self.subscribe("in", int, lambda v: self.out.append(v * self.gain.value))  # 구독 스레드에서 파라미터를 읽는다

        kros = Kros(lambda ctx: [Scaler(ctx)])
        self.addCleanup(kros.shutdown)
        node = kros.nodes[0]
        assert isinstance(node, Scaler)
        node.publish("in", 2)
        kros.context.topic_executor.join()
        def change() -> None:
            kros.set_parameter(node, "gain", 10)

        changer = threading.Thread(target=change)  # 다른 스레드에서 바꾼다
        changer.start()
        changer.join(2)
        node.publish("in", 2)
        kros.context.topic_executor.join()
        self.assertEqual(node.out, [2, 20])


class Watcher(Node):
    """다른 노드의 파라미터 변화를 parameter_events 토픽으로 지켜본다."""

    def __init__(self, context: Context) -> None:
        super().__init__(context)
        self.events: list[ParameterEvent] = []
        self.subscribe(PARAMETER_EVENTS_TOPIC_NAME, ParameterEvent, self.events.append)


class TestParameterEvents(unittest.TestCase):
    def make(self, parameters: dict[str, dict[str, bool | int | float | str]] | None = None) -> tuple[Kros, Watcher, Speedy]:
        kros = Kros(lambda ctx: [Watcher(ctx), Speedy(ctx)], parameters=parameters)
        self.addCleanup(kros.shutdown)
        watcher, speedy = kros.nodes
        assert isinstance(watcher, Watcher) and isinstance(speedy, Speedy)
        return kros, watcher, speedy

    def settle(self, kros: Kros) -> None:
        kros.context.topic_executor.join()

    def test_declaring_publishes_declared_events_with_the_starting_value(self) -> None:
        kros, watcher, _ = self.make({"Speedy": {"speed": 0.4}})
        self.settle(kros)
        declared = {e.name: e for e in watcher.events if e.node == "Speedy"}
        self.assertEqual(set(declared), {"speed", "topic", "count", "log_level"})  # log_level은 모든 노드가 자동으로 선언한다
        self.assertEqual((declared["speed"].kind, declared["speed"].old, declared["speed"].new), (ParameterEventKind.DECLARED, None, 0.4))  # 덮어쓴 시작 값
        self.assertEqual(declared["topic"].new, "cmd_vel")  # 덮어쓰지 않은 건 기본값

    def test_change_publishes_old_and_new(self) -> None:
        kros, watcher, speedy = self.make()
        self.settle(kros)
        watcher.events.clear()
        kros.set_parameter(speedy, "speed", 0.9)
        self.settle(kros)
        self.assertEqual(len(watcher.events), 1)
        event = watcher.events[0]
        self.assertEqual((event.node, event.name, event.kind, event.old, event.new), ("Speedy", "speed", ParameterEventKind.CHANGED, 0.26, 0.9))
        self.assertTrue(event.origin)  # 이 인스턴스가 낸 이벤트 표시 (자기 에코를 다시 적용하지 않는다)

    def test_every_way_of_changing_is_announced_to_other_nodes(self) -> None:
        kros, watcher, speedy = self.make()
        self.settle(kros)
        watcher.events.clear()
        speedy.speed.set(0.1)  # 손잡이로
        speedy.set_parameter("count", 5)  # 노드 이름으로
        kros.context.parameter_manager.set("Speedy", "topic", "scan")  # 매니저로 직접
        self.settle(kros)
        self.assertEqual({(e.name, e.new) for e in watcher.events}, {("speed", 0.1), ("count", 5), ("topic", "scan")})

    def test_rejected_or_wrongly_typed_changes_publish_nothing(self) -> None:
        kros = Kros(lambda ctx: [Watcher(ctx), Checked(ctx)])
        self.addCleanup(kros.shutdown)
        watcher = kros.nodes[0]
        assert isinstance(watcher, Watcher)
        self.settle(kros)
        watcher.events.clear()
        with self.assertRaises(ParameterRejectedError):
            kros.set_parameter("Checked", "limit", -1)
        with self.assertRaises(TypeError):
            kros.set_parameter("Checked", "limit", "x")
        self.settle(kros)
        self.assertEqual(watcher.events, [])

    def test_setting_the_same_value_publishes_nothing(self) -> None:
        kros, watcher, speedy = self.make()
        self.settle(kros)
        watcher.events.clear()
        speedy.speed.set(0.26)  # 이미 이 값
        self.settle(kros)
        self.assertEqual(watcher.events, [])

    def test_declared_comes_before_changed(self) -> None:
        kros, watcher, speedy = self.make()
        speedy.speed.set(2.0)
        self.settle(kros)
        kinds = [e.kind for e in watcher.events if e.node == "Speedy" and e.name == "speed"]
        self.assertEqual(kinds, [ParameterEventKind.DECLARED, ParameterEventKind.CHANGED])

    def test_parameter_manager_works_without_any_topic(self) -> None:
        manager = ParameterManager()  # 이벤트 콜백이 없어도 동작한다
        manager.declare("n", "x", 1)
        manager.set("n", "x", 2)
        self.assertEqual(manager.get("n", "x"), 2)

    def test_manager_calls_the_event_callback_directly(self) -> None:
        seen: list[ParameterEvent] = []
        manager = ParameterManager(on_event=seen.append)
        manager.declare("n", "x", 1)
        manager.set("n", "x", 2)
        self.assertEqual([(e.kind, e.old, e.new) for e in seen], [(ParameterEventKind.DECLARED, None, 1), (ParameterEventKind.CHANGED, 1, 2)])


class Checked(Node):
    def __init__(self, context: Context) -> None:
        super().__init__(context)
        self.limit = self.declare_parameter("limit", 10, validator=lambda v: v >= 0)


class TestManagersAreNotTiedTogether(unittest.TestCase):
    """매니저끼리는 서로를 모른다. 연결은 Context가 가운데서 한다 (교통정리)."""

    def test_managers_do_not_import_each_other(self) -> None:
        import pathlib
        import re

        root = pathlib.Path(__file__).parent.parent
        managers = ("topic", "service", "parameter", "timer")
        for manager in managers:
            others = [m for m in managers if m != manager] + ["action"]  # 액션은 토픽/서비스 위에 올라가는 기능이라, 아래 매니저가 위를 import하면 안 된다
            banned = re.compile(rf"^\s*(from|import)\s+kros\.({'|'.join(others)})\b", re.M)
            for f in (root / manager).rglob("*.py"):
                self.assertIsNone(banned.search(f.read_text()), f"{f}: {manager} 매니저가 다른 매니저를 import한다 (연결은 Context에서)")

    def test_context_is_the_one_that_wires_parameter_events_to_the_topic(self) -> None:
        ctx = Context()
        self.addCleanup(ctx.shutdown)
        seen: list[ParameterEvent] = []
        Node(ctx, "w").subscribe(PARAMETER_EVENTS_TOPIC_NAME, ParameterEvent, seen.append)
        Node(ctx, "n").declare_parameter("x", 1)
        ctx.topic_executor.join()
        self.assertEqual([(e.node, e.name) for e in seen], [("n", "log_level"), ("n", "x")])  # log_level은 모든 노드가 자동으로 선언한다


class TestKrosIntegration(unittest.TestCase):
    def test_kros_passes_parameters_to_the_nodes(self) -> None:
        kros = Kros(lambda ctx: [Speedy(ctx)], parameters={"Speedy": {"topic": "lidar", "count": 7}})
        self.addCleanup(kros.shutdown)
        node = kros.nodes[0]
        assert isinstance(node, Speedy)
        self.assertEqual((node.topic.value, node.count.value, node.speed.value), ("lidar", 7, 0.26))

    def test_external_set_through_the_context_manager(self) -> None:
        kros = Kros(lambda ctx: [Speedy(ctx)])
        self.addCleanup(kros.shutdown)
        node = kros.nodes[0]
        assert isinstance(node, Speedy)
        kros.context.parameter_manager.set("Speedy", "speed", 0.05)  # 노드 밖에서 바꾼다
        self.assertEqual(node.speed.value, 0.05)

    def test_typo_in_a_given_parameter_is_reported(self) -> None:
        with self.assertLogs("kros.kros", level="WARNING") as logs:
            kros = Kros(lambda ctx: [Speedy(ctx)], parameters={"Speedy": {"spede": 1.0}, "Ghost": {"x": 1}})
        self.addCleanup(kros.shutdown)
        text = "\n".join(logs.output)
        self.assertIn("Speedy.spede", text)  # 선언 안 된 파라미터
        self.assertIn("Ghost.x", text)  # 없는 노드

    def test_no_warning_when_every_given_parameter_is_declared(self) -> None:
        logger = logging.getLogger("kros.kros")
        with self.assertNoLogs(logger, level="WARNING"):
            kros = Kros(lambda ctx: [Speedy(ctx)], parameters={"Speedy": {"speed": 1.0}})
        self.addCleanup(kros.shutdown)


if __name__ == "__main__":
    unittest.main()
