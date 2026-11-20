import logging
import unittest

from kros import Kros
from kros.context import Context
from kros.node import Node
from kros.parameter import (
    ParameterAlreadyDeclaredError,
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
        kros.context.topic_manager.join()
        def change() -> None:
            kros.set_parameter(node, "gain", 10)

        changer = threading.Thread(target=change)  # 다른 스레드에서 바꾼다
        changer.start()
        changer.join(2)
        node.publish("in", 2)
        kros.context.topic_manager.join()
        self.assertEqual(node.out, [2, 20])


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
