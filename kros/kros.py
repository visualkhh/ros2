import logging
from collections.abc import Mapping
from typing import Callable, Iterable

from kros.context import Context
from kros.node import Node
from kros.parameter import ParameterValue

logger = logging.getLogger(__name__)

NodeCreate = Callable[[Context], Iterable[Node]]


class Kros:
    """부트스트랩: Context를 만들고, node_create 콜백에 넘겨 노드들을 받아 보관한다.

    parameters: 파라미터 시작 값 {노드이름: {파라미터: 값}}. 노드가 선언한 기본값보다 우선한다 (파라미터 처리는 Context의 parameter_manager가 한다).
    누구도 선언하지 않은 값(오타)은 경고를 남긴다.
    """

    def __init__(self, node_create: NodeCreate, parameters: Mapping[str, Mapping[str, ParameterValue]] | None = None) -> None:
        self.context: Context = Context(parameters)
        self.nodes: list[Node] = list(node_create(self.context))
        for unused in self.context.parameter_manager.unused_overrides():
            logger.warning("값을 줬는데 선언된 파라미터가 없다 (오타?): %s", unused)

    def get_parameter(self, node: Node | str, name: str) -> ParameterValue:
        """실행 중인 노드의 파라미터 현재 값 (node는 노드 객체 또는 이름)."""
        return self.context.parameter_manager.get(node if isinstance(node, str) else node.name, name)

    def set_parameter(self, node: Node | str, name: str, value: ParameterValue) -> None:
        """실행 중인 노드의 파라미터를 바꾼다 (node는 노드 객체 또는 이름). 노드 안에서는 바로 새 값이 보이고, 타입이 다르면 TypeError,
        검증이 거절하면 ParameterRejectedError, on_change가 있으면 불린다."""
        self.context.parameter_manager.set(node if isinstance(node, str) else node.name, name, value)

    def shutdown(self) -> None:
        self.context.shutdown()
