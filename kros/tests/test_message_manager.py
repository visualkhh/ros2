import collections
import time
import unittest
from typing import Any

from kros.message_manager import MessageManager, _Peer
from kros.msgs.std_msgs import String
from kros.msgs.std_srvs.trigger import Trigger, TriggerRequest, TriggerResponse
from kros.topic import QOS_LATCHED


def _wait_port(manager: MessageManager) -> int:
    for _ in range(100):
        server = manager._tcp_server
        if server is not None:
            return int(server.getsockname()[1])
        time.sleep(0.01)
    raise AssertionError("TCP 서버가 열리지 않았다")


class TestNetworkProtocol(unittest.TestCase):
    def test_discover_is_not_ping_ponged(self) -> None:
        """연결할 때 discover는 양쪽이 한 번씩만 보낸다 (예전에는 받을 때마다 되돌려 보내 초당 수만 건이 됐다)."""
        counts: collections.Counter[str] = collections.Counter()
        original = _Peer._write

        def counting(peer: _Peer, msg: dict[str, Any]) -> None:
            messages = (msg.get("msgs") or []) if msg.get("type") == "batch" else [msg]
            for message in messages:
                counts[str(message.get("type"))] += 1
            original(peer, msg)

        _Peer._write = counting  # type: ignore[assignment]
        self.addCleanup(setattr, _Peer, "_write", original)
        server = MessageManager(host={"ip": "127.0.0.1", "port": 0})
        self.addCleanup(server.shutdown)
        port = _wait_port(server)
        client = MessageManager(servers=[{"ip": "127.0.0.1", "port": port}])
        self.addCleanup(client.shutdown)
        time.sleep(0.5)
        self.assertLessEqual(counts["discover"], 2)

    def test_latched_topic_is_replayed_to_a_late_subscriber(self) -> None:
        """서버가 먼저 낸 지도(map)를 나중에 접속한 Viz도 받는다 (연결 시 구독 목록을 보내면 그 토픽의 이력을 보낸다)."""
        server = MessageManager(host={"ip": "127.0.0.1", "port": 0})
        self.addCleanup(server.shutdown)
        server.publish("map", String(data="LATCHED-MAP"), QOS_LATCHED)
        port = _wait_port(server)
        client = MessageManager(servers=[{"ip": "127.0.0.1", "port": port}])
        self.addCleanup(client.shutdown)
        got: list[str] = []
        client.subscribe("map", String, lambda message: got.append(message.data), QOS_LATCHED)
        for _ in range(100):
            if got:
                break
            time.sleep(0.01)
        self.assertEqual(got, ["LATCHED-MAP"])

    def test_only_subscribed_topics_are_sent(self) -> None:
        """상대가 구독하지 않은 토픽은 네트워크로 보내지 않는다."""
        server = MessageManager(host={"ip": "127.0.0.1", "port": 0})
        self.addCleanup(server.shutdown)
        port = _wait_port(server)
        client = MessageManager(servers=[{"ip": "127.0.0.1", "port": port}])
        self.addCleanup(client.shutdown)
        subscribed: list[str] = []
        client.subscribe("scan", String, lambda message: subscribed.append(message.data))
        time.sleep(0.2)  # 구독 목록이 서버에 도착할 시간
        server.publish("scan", String(data="wanted"))
        server.publish("plan", String(data="not-subscribed"))
        time.sleep(0.4)
        self.assertEqual(subscribed, ["wanted"])

    def test_service_is_forwarded_to_the_server(self) -> None:
        """로컬에 없는 서비스는 연결 너머의 서버가 처리한다 (원격 Viz의 지도 저장/불러오기)."""
        server = MessageManager(host={"ip": "127.0.0.1", "port": 0})
        self.addCleanup(server.shutdown)
        server.create_service("ping", Trigger, lambda request: TriggerResponse(success=True, message="pong"))
        port = _wait_port(server)
        client = MessageManager(servers=[{"ip": "127.0.0.1", "port": port}])
        self.addCleanup(client.shutdown)
        time.sleep(0.3)  # discover로 서비스 목록이 도착할 시간
        response = client.call_service("ping", Trigger, TriggerRequest(), timeout=2.0)
        self.assertEqual((response.success, response.message), (True, "pong"))

    def test_parameters_are_synced_when_connecting(self) -> None:
        """서버 노드의 파라미터(예: cartographer mode)를 원격 Viz가 읽고 바꿀 수 있게 연결 시 복사한다."""
        server = MessageManager(host={"ip": "127.0.0.1", "port": 0})
        self.addCleanup(server.shutdown)
        server.set_parameter_hooks(snapshot=lambda: {"cartographer": {"mode": "idle"}}, apply=lambda parameters: None)
        port = _wait_port(server)
        seen: list[dict[str, dict[str, Any]]] = []
        client = MessageManager(servers=[{"ip": "127.0.0.1", "port": port}])
        self.addCleanup(client.shutdown)
        client.set_parameter_hooks(snapshot=lambda: {}, apply=seen.append)
        for _ in range(100):
            if seen:
                break
            time.sleep(0.01)
        self.assertEqual(seen, [{"cartographer": {"mode": "idle"}}])


if __name__ == "__main__":
    unittest.main()
