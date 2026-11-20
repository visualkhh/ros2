import unittest

from kros.context import Context
from kros.node import Node


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
