"""kros의 메시지 관리: 토픽/서비스/액션 실행기를 한 곳에 묶고, 네트워크(TCP) 서버/클라이언트를 관리한다.

- 로컬 모드 (host/servers 없음): 지금까지의 동작과 같다.
- 네트워크 모드: host={ip,port} (로컬 수신) 또는 servers=[{ip,port}] (원격 접속)를 주면 TCP로 다른 kros 인스턴스와 토픽/서비스/액션을 주고받는다.

프로토콜 (TCP, pickle 직렬화, 길이 접두사):
  프레임: [4바이트 길이][1바이트 압축 플래그][payload]  (플래그 1이면 payload는 zlib)
  {"type": "heartbeat", "ts": float}
  {"type": "discover", "topics": [str], "services": [str], "actions": [str], "subscriptions": [str]}
  {"type": "subscribe", "topics": [str]}                 # 내 구독 목록이 바뀌었다 (수신 측은 latched 재전송)
  {"type": "parameters", "parameters": {노드: {이름: 값}}}  # 연결 시 상대 노드들의 파라미터 현재 값
  {"type": "topic", "name": str, "msg": Any, "latched": bool}
  {"type": "batch", "msgs": [...]}                       # 여러 메시지를 한 프레임에 묶어 보낸다
  {"type": "service_req", "id": int, "name": str, "req": Any}
  {"type": "service_res", "id": int, "res": Any, "error": str|None}
  {"type": "action_goal", "id": int, "name": str, "goal": Any}
  {"type": "action_feedback", "id": int, "feedback": Any}
  {"type": "action_result", "id": int, "result": Any, "error": str|None}

성능 설계:
- 보내기는 연결마다 전용 스레드가 맡는다 → 발행 스레드(시뮬레이션/컨트롤러)가 sendall에 막히지 않는다.
- 20ms마다 모아서 한 프레임에 묶는다 (작은 패킷 수천 개 → 큰 패킷 몇 개).
- 센서/지도 같은 텔레메트리는 배치 안에서 "최신 값"만 남긴다 (느린 회선이면 자동으로 따라잡는다).
- 1KB 넘는 프레임(지도/costmap)은 zlib으로 압축한다 (대부분 0/-1이라 수십 배 줄어든다).
- discover는 연결당 한 번만 보내고 절대 되돌려 보내지 않는다 (이전 버전의 무한 핑퐁 제거).
- 구독하지 않은 토픽은 보내지 않는다 (수신 측 subscribe 목록으로 필터).
"""
from __future__ import annotations

import pickle
import socket
import struct
import threading
import time
import zlib
from collections import deque
from concurrent.futures import Future
from typing import Any, Callable, Mapping

from kros.action import ActionExecutor, ActionType
from kros.parameter import ParameterValue
from kros.service import ServiceExecutor, ServiceType
from kros.topic import Durability, TopicExecutor, QOS_DEFAULT, QoS, QOS_LATCHED
from kros.timer import TimeSource, TimerManager

FLUSH_INTERVAL = 0.02  # 초: 이만큼 모아서 한 프레임에 보낸다
COMPRESS_MIN = 1024  # 바이트: 이보다 큰 프레임은 zlib 압축
MAX_EVENTS = 500  # 연결마다 쌓아 두는 순서 보장 메시지 상한 (넘치면 오래된 것부터 버림)

# 토픽별 최소 전송 간격 (초). 없으면 매번 보낸다.
# tf는 발행자가 여럿(로봇 상태/오도메트리)이라 시간으로 자르면 한쪽이 계속 밀릴 수 있어 넣지 않는다 (작고, 배치로 묶인다).
THROTTLE = {
    "scan": 0.1,
    "odom": 0.1,
    "imu": 0.2,
    "odometry/filtered": 0.1,
    "local_plan": 0.1,
    "global_costmap/costmap": 0.5,
    "local_costmap/costmap": 0.5,
}

# 배치 안에서 최신 것만 남겨도 되는 토픽 (tf는 프레임별 발행자가 달라서 제외)
COALESCE = frozenset(
    {
        "scan",
        "map",
        "odom",
        "odometry/filtered",
        "imu",
        "local_plan",
        "global_costmap/costmap",
        "local_costmap/costmap",
    }
)


class _Peer:
    """TCP 연결 하나. 송신 전용 스레드가 큐를 비우며 배치/압축해서 보낸다.

    push()는 큐에 넣기만 하므로 발행 스레드를 절대 막지 않는다.
    """

    def __init__(self, conn: socket.socket, addr: Any) -> None:
        self.conn = conn
        self.addr = addr
        self.subscriptions: set[str] | None = None  # None = 아직 모름 (모두 전송)
        self.services: set[str] = set()  # 상대가 제공하는 서비스 (discover로 받는다)
        self.alive = True
        self._cond = threading.Condition()
        self._events: deque[dict[str, Any]] = deque(maxlen=MAX_EVENTS)
        self._latest: dict[str, dict[str, Any]] = {}
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()

    def push(self, msg: dict[str, Any], coalesce_key: str | None = None) -> None:
        with self._cond:
            if not self.alive:
                return
            if coalesce_key is not None:
                self._latest[coalesce_key] = msg  # 같은 키의 이전 값은 덮어쓴다
            else:
                self._events.append(msg)
            self._cond.notify()

    def close(self) -> None:
        with self._cond:
            if not self.alive:
                return
            self.alive = False
            self._events.clear()
            self._latest.clear()
            self._cond.notify()
        try:
            self.conn.close()
        except OSError:
            pass

    def _send_loop(self) -> None:
        while True:
            with self._cond:
                while self.alive and not self._events and not self._latest:
                    self._cond.wait(0.5)
                if not self.alive:
                    return
            time.sleep(FLUSH_INTERVAL)  # 잠깐 모아서 패킷 수를 줄인다
            with self._cond:
                if not self.alive:
                    return
                events = list(self._events)
                self._events.clear()
                latest = list(self._latest.values())
                self._latest.clear()
            batch = events + latest
            if len(batch) == 1:
                self._write(batch[0])
            elif batch:
                self._write({"type": "batch", "msgs": batch})

    def _write(self, msg: dict[str, Any]) -> None:
        try:
            data = pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL)
            flag = 0
            if len(data) >= COMPRESS_MIN:
                compressed = zlib.compress(data, 3)
                if len(compressed) < len(data):
                    data = compressed
                    flag = 1
            self.conn.sendall(struct.pack("!IB", len(data), flag) + data)
        except OSError:
            self.close()


class MessageManager:
    """토픽/서비스/액션 실행기 + (선택) TCP 서버/클라이언트를 묶는 매니저.

    Kros 생성 시 host/servers를 넘기면 네트워크 모드로 동작한다.
    """

    def __init__(
        self,
        host: Mapping[str, Any] | None = None,
        servers: list[Mapping[str, Any]] | None = None,
        time_source: TimeSource | None = None,
    ) -> None:
        self.host = host  # {"ip": str, "port": int} or None
        self.servers = servers or []  # [{"ip": str, "port": int}, ...]
        self.network_mode = bool(host or servers)

        # 실행기 (기존 매니저와 동일)
        self.topic_executor: TopicExecutor = TopicExecutor()
        self.service_executor: ServiceExecutor = ServiceExecutor()
        self.action_executor: ActionExecutor = ActionExecutor(self.topic_executor, self.service_executor)
        self.timer_manager: TimerManager = TimerManager(time_source)

        # 네트워크 상태
        self._tcp_server: socket.socket | None = None
        self._peers: list[_Peer] = []
        self._peers_lock = threading.Lock()
        self._lock = threading.Lock()  # 서비스 요청 id/대기 future 보호
        self._local = threading.local()  # 수신 스레드가 다시 브로드캐스트하지 않게 (스레드별 플래그)
        self._running = True
        self._heartbeat_interval = 5.0
        self._remote_topics: set[str] = set()
        self._remote_services: set[str] = set()
        self._remote_actions: set[str] = set()
        self._service_counter = 0
        self._action_counter = 0
        self._pending_services: dict[int, Any] = {}
        self._pending_actions: dict[int, Any] = {}
        self._last_broadcast: dict[str, float] = {}
        self._last_subs: list[str] = []
        self._parameter_snapshot: Callable[[], dict[str, dict[str, Any]]] | None = None
        self._parameter_apply: Callable[[dict[str, dict[str, Any]]], None] | None = None
        self._pending_parameters: dict[str, dict[str, Any]] = {}

        self._install_tracking()

        if self.network_mode:
            self._start_network()
            threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    # ---- 로컬 실행기 계측 (발행/구독을 가로채 네트워크와 연결) ----

    def _install_tracking(self) -> None:
        orig_publish = self.topic_executor.publish

        def publish(topic: str, msg: Any, qos: QoS = QOS_DEFAULT) -> None:
            orig_publish(topic, msg, qos)
            if self.network_mode and not getattr(self._local, "receiving", False):
                self._broadcast_topic(topic, msg, qos)

        self.topic_executor.publish = publish  # type: ignore[method-assign]

        orig_subscribe = self.topic_executor.subscribe

        def subscribe(topic: str, msg_type: type, callback: Callable[[Any], None], qos: QoS = QOS_DEFAULT) -> Any:
            sub = orig_subscribe(topic, msg_type, callback, qos)
            self._subscriptions_changed()
            return sub

        self.topic_executor.subscribe = subscribe  # type: ignore[method-assign]

        # 로컬에 없는 서비스는 네트워크 너머의 서버로 보낸다 (원격 Viz의 지도 저장/불러오기 등)
        orig_call_async = self.service_executor.call_async

        def call_async(name: str, srv_type: Any, request: Any) -> Future[Any]:
            try:
                return orig_call_async(name, srv_type, request)
            except LookupError:
                if not self.network_mode:
                    raise
                return self._remote_call(name, request)

        self.service_executor.call_async = call_async  # type: ignore[method-assign]

    def _remote_call(self, name: str, request: Any) -> Future[Any]:
        with self._peers_lock:
            peer = next((p for p in self._peers if p.alive and name in p.services), None)
        if peer is None:
            raise LookupError(f"no server for service {name!r}")
        future: Future[Any] = Future()
        with self._lock:
            self._service_counter += 1
            request_id = self._service_counter
            self._pending_services[request_id] = future
        future.add_done_callback(lambda unused: self._pending_services.pop(request_id, None))
        peer.push({"type": "service_req", "id": request_id, "name": name, "req": request})
        return future

    def _broadcast_topic(self, topic: str, msg: Any, qos: QoS) -> None:
        now = time.monotonic()
        interval = THROTTLE.get(topic, 0.0)
        if interval > 0.0:
            if now - self._last_broadcast.get(topic, 0.0) < interval:
                return
            self._last_broadcast[topic] = now
        latched = qos.durability is Durability.TRANSIENT_LOCAL
        self._broadcast({"type": "topic", "name": topic, "msg": msg, "latched": latched}, topic=topic)

    def set_parameter_hooks(
        self,
        snapshot: Callable[[], dict[str, dict[str, Any]]],
        apply: Callable[[dict[str, dict[str, Any]]], None],
    ) -> None:
        """파라미터 동기화 훅. 연결할 때 snapshot()을 보내고, 받은 표는 apply()로 반영한다."""
        self._parameter_snapshot = snapshot
        self._parameter_apply = apply
        if self._pending_parameters:  # 연결이 먼저 맺어져 훅보다 일찍 도착한 표가 있으면 지금 반영한다
            pending, self._pending_parameters = self._pending_parameters, {}
            apply(pending)

    # ---- 연결 관리 ----

    def _heartbeat_loop(self) -> None:
        while self._running:
            time.sleep(self._heartbeat_interval)
            if not self._running:
                break
            self._broadcast({"type": "heartbeat", "ts": time.time()})

    def _start_network(self) -> None:
        if self.host:
            ip = self.host.get("ip", "0.0.0.0")
            port = int(self.host.get("port", 9000))
            self._tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._tcp_server.bind((ip, port))
            self._tcp_server.listen(8)
            self._tcp_server.settimeout(0.5)  # shutdown 때 accept가 0.5초 안에 깨어나 포트를 놓는다
            threading.Thread(target=self._accept_loop, daemon=True).start()
        for c in self.servers:
            ip = c.get("ip", "127.0.0.1")
            port = int(c.get("port", 9000))
            threading.Thread(target=self._connect_loop, args=(ip, port), daemon=True).start()

    def _accept_loop(self) -> None:
        while self._running:
            try:
                server = self._tcp_server
                if server is None:
                    break
                conn, addr = server.accept()
            except TimeoutError:  # settimeout: 주기적으로 _running을 확인한다
                continue
            except OSError:
                break
            self._register_conn(conn, addr)

    def _connect_loop(self, ip: str, port: int) -> None:
        while self._running:
            try:
                conn = socket.create_connection((ip, port), timeout=5.0)
                conn.settimeout(None)
                peer = self._register_conn(conn, (ip, port))
                while self._running and peer.alive:  # 서버가 꺼지면 다시 연결한다 (Viz를 켜 둔 채 서버만 재시작)
                    time.sleep(0.2)
            except OSError:
                time.sleep(2.0)

    def _register_conn(self, conn: socket.socket, addr: Any) -> _Peer:
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # 작은 메시지의 지연(Nagle) 제거
        except OSError:
            pass
        peer = _Peer(conn, addr)
        with self._peers_lock:
            self._peers.append(peer)
        # discover는 연결당 정확히 한 번. 받은 쪽은 절대 되돌려 보내지 않는다 (핑퐁 방지).
        peer.push(
            {
                "type": "discover",
                "topics": self._local_topics(),
                "services": self._local_services(),
                "actions": self._local_actions(),
                "subscriptions": self._local_subscriptions(),
            }
        )
        if self._parameter_snapshot is not None:
            try:
                peer.push({"type": "parameters", "parameters": self._parameter_snapshot()})
            except Exception:
                pass  # 파라미터 표를 못 만들면 연결은 계속한다
        threading.Thread(target=self._recv_loop, args=(peer,), daemon=True).start()
        return peer

    def _recv_loop(self, peer: _Peer) -> None:
        buf = bytearray()
        try:
            while self._running and peer.alive:
                data = peer.conn.recv(65536)
                if not data:
                    break
                buf += data
                while len(buf) >= 5:
                    length, flag = struct.unpack_from("!IB", buf)
                    end = 5 + length
                    if len(buf) < end:
                        break
                    payload = bytes(buf[5:end])
                    del buf[:end]
                    if flag:
                        payload = zlib.decompress(payload)
                    self._handle_msg(peer, pickle.loads(payload))
        except (OSError, pickle.UnpicklingError, EOFError, zlib.error) as e:
            import logging

            logging.warning("recv_loop error: %s", e)
        finally:
            with self._peers_lock:
                if peer in self._peers:
                    self._peers.remove(peer)
            peer.close()

    def _broadcast(self, msg: dict[str, Any], topic: str | None = None, exclude: _Peer | None = None) -> None:
        with self._peers_lock:
            peers = list(self._peers)
        for peer in peers:
            if peer is exclude:
                continue
            if topic is not None and peer.subscriptions is not None and topic not in peer.subscriptions:
                continue  # 상대가 구독하지 않은 토픽은 보내지 않는다
            peer.push(msg, coalesce_key=topic if topic in COALESCE else None)

    def _handle_msg(self, peer: _Peer, msg: dict[str, Any]) -> None:
        mtype = msg.get("type")
        if mtype == "batch":
            for inner in msg.get("msgs") or []:
                self._handle_msg(peer, inner)
        elif mtype == "heartbeat":
            pass  # 상대가 살아 있음
        elif mtype == "discover":
            for t in msg.get("topics") or []:
                self._remote_topics.add(t)
            for s in msg.get("services") or []:
                self._remote_services.add(s)
                peer.services.add(str(s))
            for a in msg.get("actions") or []:
                self._remote_actions.add(a)
            if "subscriptions" in msg:
                self._update_subscriptions(peer, {str(t) for t in msg.get("subscriptions") or []})
        elif mtype == "service_req":
            self._handle_service_req(peer, msg)
        elif mtype == "service_res":
            self._handle_service_res(msg)
        elif mtype == "subscribe":
            self._update_subscriptions(peer, {str(t) for t in msg.get("topics") or []})
        elif mtype == "parameters":
            parameters = msg.get("parameters")
            if isinstance(parameters, dict):
                if self._parameter_apply is not None:
                    try:
                        self._parameter_apply(parameters)
                    except Exception:
                        pass  # 잘못된 표는 무시 (연결은 유지)
                else:
                    self._pending_parameters.update(parameters)  # 훅이 준비되면 반영한다
        elif mtype == "topic":
            name = msg.get("name")
            if name is not None:
                qos = QOS_LATCHED if msg.get("latched") else QOS_DEFAULT
                # 네트워크에서 받은 메시지를 로컬 토픽으로 전달한다 (같은 스레드에서 재브로드캐스트 금지)
                self._local.receiving = True
                try:
                    self.topic_executor.publish(name, msg.get("msg"), qos)
                finally:
                    self._local.receiving = False
        # TODO: action_goal, action_feedback, action_result

    def _handle_service_req(self, peer: _Peer, msg: dict[str, Any]) -> None:
        """상대가 이쪽의 서비스를 부른다: 실행하고 같은 연결로 응답한다."""
        request_id = msg.get("id")
        name = msg.get("name")
        if not isinstance(request_id, int) or not isinstance(name, str):
            return
        with self.service_executor._lock:
            server = self.service_executor._servers.get(name)
        if server is None:
            peer.push({"type": "service_res", "id": request_id, "res": None, "error": f"no server for service {name!r}"})
            return
        try:
            future = server.submit(msg.get("req"))
        except RuntimeError as error:  # 서비스가 막 닫혔다
            peer.push({"type": "service_res", "id": request_id, "res": None, "error": str(error)})
            return

        def done(finished: Future[Any]) -> None:
            try:
                response = finished.result()
            except Exception as error:  # 핸들러 예외는 문자열로 돌려준다
                peer.push({"type": "service_res", "id": request_id, "res": None, "error": str(error)})
            else:
                peer.push({"type": "service_res", "id": request_id, "res": response, "error": None})

        future.add_done_callback(done)

    def _handle_service_res(self, msg: dict[str, Any]) -> None:
        request_id = msg.get("id")
        if not isinstance(request_id, int):
            return
        with self._lock:
            future = self._pending_services.pop(request_id, None)
        if future is None or future.done():
            return
        error = msg.get("error")
        if error:
            future.set_exception(RuntimeError(str(error)))
        else:
            future.set_result(msg.get("res"))

    def _update_subscriptions(self, peer: _Peer, topics: set[str]) -> None:
        new = topics - (peer.subscriptions or set())
        peer.subscriptions = topics
        self._send_latched(peer, new)

    def _send_latched(self, peer: _Peer, topics: set[str]) -> None:
        """새로 구독한 토픽의 latched 메시지를 보낸다 (지도/costmap/robot_description/tf_static).

        tf_static은 depth=100 이력에 정적 변환이 나뉘어 들어 있으므로 마지막 하나만 보내면 안 된다 (전부 보낸다).
        """
        if not topics:
            return
        history = getattr(self.topic_executor, "_history", {})
        lock = getattr(self.topic_executor, "_lock", None)
        for topic in topics:
            if lock is not None:
                with lock:  # 발행 스레드가 deque에 추가하는 중일 수 있다
                    entries = list(history.get(topic, ()))
            else:
                entries = list(history.get(topic, ()))
            for message in entries:
                peer.push({"type": "topic", "name": topic, "msg": message, "latched": True}, coalesce_key=topic if topic in COALESCE else None)

    def _subscriptions_changed(self) -> None:
        if not self.network_mode:
            return
        topics = self._local_subscriptions()
        if topics == self._last_subs:
            return
        self._last_subs = topics
        self._broadcast({"type": "subscribe", "topics": topics})

    def _local_topics(self) -> list[str]:
        types = getattr(self.topic_executor, "_types", {})
        return list(types.keys())

    def _local_services(self) -> list[str]:
        servers = getattr(self.service_executor, "_servers", {})
        return list(servers.keys())

    def _local_actions(self) -> list[str]:
        servers = getattr(self.action_executor, "_servers", {})
        return list(servers.keys())

    def _local_subscriptions(self) -> list[str]:
        subs = getattr(self.topic_executor, "_subs", {})
        return sorted(str(name) for name, entries in subs.items() if entries)

    def shutdown(self) -> None:
        self._running = False
        if self._tcp_server is not None:
            try:
                self._tcp_server.close()
            except OSError:
                pass
        with self._peers_lock:
            peers = list(self._peers)
            self._peers.clear()
        for peer in peers:
            peer.close()
        self.timer_manager.shutdown()
        self.action_executor.shutdown()
        self.service_executor.shutdown()
        self.topic_executor.shutdown()

    # ---- 토픽 (로컬 우선, 없으면 네트워크) ----

    def publish(self, topic: str, msg: Any, qos: QoS = QOS_DEFAULT) -> None:
        # topic_executor.publish가 _install_tracking에서 네트워크 브로드캐스트까지 처리한다 (이중 전송 금지)
        self.topic_executor.publish(topic, msg, qos)

    def subscribe(self, topic: str, msg_type: type, callback: Callable[[Any], None], qos: QoS = QOS_DEFAULT) -> Any:
        return self.topic_executor.subscribe(topic, msg_type, callback, qos)

    # ---- 서비스 ----

    def create_service(self, *args: Any, **kwargs: Any) -> Any:
        return self.service_executor.create_service(*args, **kwargs)

    def destroy_service(self, *args: Any, **kwargs: Any) -> Any:
        return self.service_executor.destroy_service(*args, **kwargs)

    def call_service(self, *args: Any, **kwargs: Any) -> Any:
        return self.service_executor.call(*args, **kwargs)

    def call_service_async(self, *args: Any, **kwargs: Any) -> Any:
        return self.service_executor.call_async(*args, **kwargs)

    # ---- 액션 ----

    def create_action_server(self, *args: Any, **kwargs: Any) -> Any:
        return self.action_executor.create_server(*args, **kwargs)

    def destroy_action_server(self, *args: Any, **kwargs: Any) -> Any:
        return self.action_executor.destroy_server(*args, **kwargs)

    def create_action_client(self, *args: Any, **kwargs: Any) -> Any:
        return self.action_executor.create_client(*args, **kwargs)

    def destroy_action_client(self, *args: Any, **kwargs: Any) -> Any:
        return self.action_executor.destroy_client(*args, **kwargs)
