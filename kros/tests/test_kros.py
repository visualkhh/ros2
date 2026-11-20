import unittest

from kros import Kros
from kros.context import Context
from kros.nodes import Node


class TestKros(unittest.TestCase):
    def test_callback_gets_context_and_nodes_share_it(self) -> None:
        got: list[Context] = []

        def create(ctx: Context) -> list[Node]:
            got.append(ctx)
            return [Node(ctx, "a"), Node(ctx, "b")]

        k = Kros(create)
        self.assertIs(got[0], k.context)
        self.assertEqual([n.name for n in k.nodes], ["a", "b"])
        self.assertTrue(all(n.context is k.context for n in k.nodes))

    def test_pub_sub_between_nodes_via_manager(self) -> None:
        k = Kros(lambda ctx: [Node(ctx, "pub"), Node(ctx, "sub")])
        pub, sub = k.nodes
        got: list[str] = []
        sub.subscribe("chat", str, got.append)
        pub.publish("chat", "hello")
        pub.publish("other", "ignored")
        k.shutdown()
        self.assertEqual(got, ["hello"])
