from collections.abc import Mapping

from kros.action import ActionManager
from kros.parameter import PARAMETER_EVENTS_TOPIC_NAME, ParameterEvent, ParameterManager, ParameterValue
from kros.service import ServiceManager
from kros.topic import TopicManager


class Context:
    """노드들이 공유하는 묶음: 토픽/서비스/액션/파라미터 매니저, 그리고 노드 이름 목록(중복 방지). 매니저끼리의 연결은 여기서 한다."""

    def __init__(self, parameters: Mapping[str, Mapping[str, ParameterValue]] | None = None) -> None:
        self.topic_manager: TopicManager = TopicManager()
        self.service_manager: ServiceManager = ServiceManager()
        self.action_manager: ActionManager = ActionManager(self.topic_manager, self.service_manager)  # 액션은 토픽+서비스 위에 올라간다
        # parameters: {노드이름: {파라미터: 값}} 시작 값. 선언/변경은 parameter_events 토픽으로 알린다 (파라미터 매니저는 토픽을 모르고 콜백만 부른다)
        self.parameter_manager: ParameterManager = ParameterManager(parameters, on_event=self._publish_parameter_event)
        self._node_names: set[str] = set()

    def _publish_parameter_event(self, event: ParameterEvent) -> None:
        self.topic_manager.publish(PARAMETER_EVENTS_TOPIC_NAME, event)

    def register_node(self, name: str) -> None:
        """노드 이름을 등록한다. 같은 Context에서 이름이 겹치면 ValueError."""
        if name in self._node_names:
            raise ValueError(f"노드 이름이 겹친다: {name!r} (같은 클래스를 여럿 만들면 name=을 따로 줘라)")
        self._node_names.add(name)

    def shutdown(self) -> None:
        self.action_manager.shutdown()  # 액션 먼저: 액션은 서비스와 토픽 위에서 돈다
        self.service_manager.shutdown()  # 서비스 다음: 핸들러가 publish할 수 있으므로
        self.topic_manager.shutdown()
