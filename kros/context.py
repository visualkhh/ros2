import os
import uuid
from collections.abc import Mapping
from dataclasses import replace

from kros.message_manager import MessageManager
from kros.parameter import PARAMETER_EVENTS_TOPIC_NAME, ParameterEvent, ParameterManager, ParameterValue
from kros.topic import QOS_DEFAULT
from kros.timer import TimeSource


class Context:
    """노드들이 공유하는 묶음: 메시지 매니저(토픽/서비스/액션/타이머 실행기 + 네트워크)와 파라미터 매니저."""

    def __init__(
        self,
        parameters: Mapping[str, Mapping[str, ParameterValue]] | None = None,
        time_source: TimeSource | None = None,
        host: Mapping[str, object] | None = None,
        servers: list[Mapping[str, object]] | None = None,
    ) -> None:
        self.message_manager: MessageManager = MessageManager(host=host, servers=servers, time_source=time_source)
        # 하위 호환: 기존 코드가 context.topic_executor 등을 직접 쓰는 경우
        self.topic_executor = self.message_manager.topic_executor
        self.service_executor = self.message_manager.service_executor
        self.action_executor = self.message_manager.action_executor
        self.timer_manager = self.message_manager.timer_manager
        # parameters: {노드이름: {파라미터: 값}} 시작 값. 선언/변경은 parameter_events 토픽으로 알린다 (파라미터 매니저는 토픽을 모르고 콜백만 부른다)
        self.parameter_manager: ParameterManager = ParameterManager(parameters, on_event=self._publish_parameter_event)
        self._node_names: set[str] = set()
        self._applying_remote_param = False
        # 이 인스턴스가 낸 파라미터 이벤트의 출처 id: 토픽으로 되돌아온 자기 이벤트를 다시 적용하지 않는다
        self._origin = f"{os.getpid()}-{uuid.uuid4().hex[:8]}"
        # 네트워크: 연결할 때 서버의 파라미터를 받아 오고, 받은 값을 로컬에 반영한다
        self.message_manager.set_parameter_hooks(snapshot=self.parameter_manager.all_declared, apply=self._adopt_parameters)
        # 네트워크에서 받은 파라미터 변경을 로컬에 적용한다
        self.topic_executor.subscribe(PARAMETER_EVENTS_TOPIC_NAME, ParameterEvent, self._apply_remote_parameter_event, QOS_DEFAULT)

    def _adopt_parameters(self, parameters: dict[str, dict[str, ParameterValue]]) -> None:
        """연결 시 받은 원격 파라미터 표를 로컬에 반영한다 (원격 Viz가 서버 노드의 값을 읽고 바꿀 수 있게)."""
        for node, values in parameters.items():
            for name, value in values.items():
                try:
                    self.parameter_manager.adopt(node, name, value)
                except TypeError:
                    pass  # 지원하지 않는 타입은 무시

    def _publish_parameter_event(self, event: ParameterEvent) -> None:
        if self._applying_remote_param:
            return  # 원격에서 받은 변경은 다시 브로드캐스트하지 않는다
        self.message_manager.publish(PARAMETER_EVENTS_TOPIC_NAME, replace(event, origin=self._origin))

    def _apply_remote_parameter_event(self, event: ParameterEvent) -> None:
        if event.origin == self._origin:
            return  # 내가 낸 이벤트가 로컬 토픽으로 되돌아온 것 (이미 반영돼 있다)
        # 원격에서 온 파라미터 변경을 로컬에 적용한다
        self._applying_remote_param = True
        try:
            self.parameter_manager.set(event.node, event.name, event.new)
        except Exception:
            pass  # 이쪽에 없는 노드/파라미터, 타입 불일치 등
        finally:
            self._applying_remote_param = False

    def register_node(self, name: str) -> None:
        """노드 이름을 등록한다. 같은 Context에서 이름이 겹치면 ValueError."""
        if name in self._node_names:
            raise ValueError(f"노드 이름이 겹친다: {name!r} (같은 클래스를 여럿 만들면 name=을 따로 줘라)")
        self._node_names.add(name)

    def shutdown(self) -> None:
        self.message_manager.shutdown()
        # parameter_manager는 토픽을 모르므로 별도 shutdown 불필요