from kros.topic import TopicManager
from kros.service import ServiceManager


class Context:
    """노드들이 공유하는 통신 수단 묶음: 토픽 매니저 + 서비스 매니저."""

    def __init__(self) -> None:
        self.topic_manager: TopicManager = TopicManager()
        self.service_manager: ServiceManager = ServiceManager()

    def shutdown(self) -> None:
        self.service_manager.shutdown()  # 서비스 먼저: 핸들러가 publish할 수 있으므로
        self.topic_manager.shutdown()
