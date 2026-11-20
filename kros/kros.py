from typing import Callable, Iterable

from kros.context import Context
from kros.node import Node

NodeCreate = Callable[[Context], Iterable[Node]]


class Kros:
    """부트스트랩: Context를 만들고, node_create 콜백에 넘겨 노드들을 받아 보관한다."""

    def __init__(self, node_create: NodeCreate) -> None:
        self.context: Context = Context()
        self.nodes: list[Node] = list(node_create(self.context))

    def shutdown(self) -> None:
        self.context.shutdown()
