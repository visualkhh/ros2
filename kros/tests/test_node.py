import unittest

from kros.context import Context
from kros.nodes import Node


class TestNode(unittest.TestCase):
    def test_name_and_context(self) -> None:
        ctx = Context()
        n = Node(ctx, "a")
        self.assertEqual((n.name, n.context), ("a", ctx))

    def test_repr(self) -> None:
        self.assertEqual(repr(Node(Context(), "a")), "Node('a')")

    def test_pub_sub_unsub_typed(self) -> None:
        ctx = Context()
        self.addCleanup(ctx.shutdown)
        got: list[int] = []
        sub_node, pub_node = Node(ctx, "s"), Node(ctx, "p")
        sub = sub_node.subscribe("n", int, got.append)
        pub_node.publish("n", 7)
        ctx.topic_manager.join()
        sub_node.unsubscribe(sub)
        pub_node.publish("n", 8)
        ctx.topic_manager.join()
        self.assertEqual(got, [7])


class TestProvidedNodesLayout(unittest.TestCase):
    """kros가 제공하는 노드는 nodes/에 모으고, 그 노드가 쓰는 도구 패키지는 nodes/를 실행 때 import하지 않는다 (순환 방지: 타입 표기용 TYPE_CHECKING 안은 허용)."""

    root = __import__("pathlib").Path(__file__).parent.parent

    def test_provided_nodes_live_in_nodes(self) -> None:
        import kros.nodes as provided

        self.assertEqual(sorted(provided.__all__), ["DiagnosticAggregator", "DiffDriveOdometry", "Node", "RobotStatePublisher", "TfEcho"])
        for name in provided.__all__:
            self.assertTrue(issubclass(getattr(provided, name), Node), name)

    def test_tools_do_not_import_nodes_at_runtime(self) -> None:
        import re

        banned = re.compile(r"^(from|import)\s+kros\.nodes\b", re.M)  # 들여쓴 import(TYPE_CHECKING 안)는 실행 때 일어나지 않는다
        for package in ("tf", "diagnostics", "urdf", "log", "topic", "service", "action", "parameter", "timer"):
            for f in (self.root / package).rglob("*.py"):
                self.assertIsNone(banned.search(f.read_text()), f"{f}는 nodes/를 실행 때 import한다 (의존 방향은 nodes -> 도구)")

    def test_tools_packages_no_longer_export_the_nodes(self) -> None:
        import kros.diagnostics
        import kros.tf

        self.assertFalse(hasattr(kros.tf, "RobotStatePublisher"))
        self.assertFalse(hasattr(kros.diagnostics, "DiagnosticAggregator"))
