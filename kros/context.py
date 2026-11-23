from collections.abc import Mapping

from kros.message_manager import MessageManager
from kros.parameter import PARAMETER_EVENTS_TOPIC_NAME, ParameterEvent, ParameterManager, ParameterValue
from kros.timer import TimeSource


class Context:
    """노드들이 공유하는 묶음: 메시지 매니저(토픽/서비스/액션/타이머 실행기 + 네트워크)와 파라미터 매니저."""

    def __init__(
        self,
        parameters: Mapping[str, Mapping[str, ParameterValue]] | None = None,
        time_source: TimeSource | None = None,
        server: Mapping[str, object] | None = None,
        clients: list[Mapping[str, object]] | None = None,
    ) -> None:
        self.message_manager: MessageManager = MessageManager(server=server, clients=clients, time_source=time_source)
        # 하위 호환: 기존 코드가 context.topic_manager 등을 직접 쓰는 경우
        self.topic_manager = self.message_manager.topic_executor
        self.service_manager = self.message_manager.service_executor
        self.action_manager = self.message_manager.action_executor
        self.timer_manager = self.message_manager.timer_manager
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
        self.message_manager.shutdown()
        # parameter_manager는 토픽을 모르므로 별도 shutdown 불필요