"""kros의 메시지 관리: 토픽/서비스/액션 실행기를 한 곳에 묶고, 네트워크(TCP) 서버/클라이언트를 관리한다.

- 로컬 모드 (server/clients 없음): 지금까지의 동작과 같다.
- 네트워크 모드: host={ip,port} (로컬 수신) 또는 servers=[{ip,port}] (원격 접속)를 주면 TCP로 다른 kros 인스턴스와 토픽/서비스/액션을 주고받는다.
- MessageManager를 통해 구독/발행/서비스/액션을 하면, 먼저 로컬에서 찾고, 없으면 네트워크에서 찾는다.

프로토콜 (TCP, pickle 직렬화, 길이 접두사):
  {"type": "heartbeat", "node": str, "ts": float}
  {"type": "discover", "topics": [str], "services": [str], "actions": [str]}
  {"type": "topic", "name": str, "msg": Any}          # 발행된 토픽 메시지
  {"type": "service_req", "id": int, "name": str, "req": Any}
  {"type": "service_res", "id": int, "res": Any, "error": str|None}
  {"type": "action_goal", "id": int, "name": str, "goal": Any}
  {"type": "action_feedback", "id": int, "feedback": Any}
  {"type": "action_result", "id": int, "result": Any, "error": str|None}
"""
from __future__ import annotations

import pickle
import socket
import struct
import threading
import time
from typing import Any, Callable, Mapping

from kros.action import ActionExecutor, ActionType
from kros.parameter import ParameterValue
from kros.service import ServiceExecutor, ServiceType
from kros.topic import TopicExecutor, QOS_DEFAULT, QoS, QOS_LATCHED
from kros.timer import TimeSource, TimerManager


class MessageManager:
    """토픽/서비스/액션 실행기 + (선택) TCP 서버/클라이언트를 묶는 매니저.

    Kros 생성 시 server/clients를 넘기면 네트워크 모드로 동작한다.
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

        # 토픽 발행 시 네트워크 브로드캐스트를 자동으로 하도록 publish를 래핑한다
        _orig_publish = self.topic_executor.publish
        def _publish_with_broadcast(topic: str, msg: Any, qos: QoS = QOS_DEFAULT) -> None:
            _orig_publish(topic, msg, qos)
            if self.network_mode and not getattr(self, "_receiving", False):
                now = time.monotonic()
                if not hasattr(self, "_last_broadcast"):
                    self._last_broadcast: dict[str, float] = {}
                last = self._last_broadcast.get(topic, 0.0)
                throttle_topics = {"scan", "odom", "tf", "tf_static", "imu", "odometry/filtered", "local_costmap/costmap", "global_costmap/costmap"}
                min_interval = 0.1 if topic in throttle_topics else 0.0
                if now - last >= min_interval:
                    self._last_broadcast[topic] = now
                    self._broadcast({"type": "topic", "name": topic, "msg": msg})
        self.topic_executor.publish = _publish_with_broadcast  # type: ignore[method-assign]

        # 네트워크
        self._lock = threading.Lock()
        self._tcp_server: socket.socket | None = None
        self._connections: list[socket.socket] = []  # 활성 TCP 연결
        self._conn_lock = threading.Lock()
        self._running = True
        self._heartbeat_interval = 5.0  # 초
        self._node_name = "kros"
        self._remote_topics: set[str] = set()
        self._remote_services: set[str] = set()
        self._remote_actions: set[str] = set()
        self._receiving = False
        self._service_counter = 0
        self._action_counter = 0
        self._pending_services: dict[int, Any] = {}  # id -> result future
        self._pending_actions: dict[int, Any] = {}

        if self.network_mode:
            self._start_network()
            # 하트비트: 주기적으로 살아 있음을 알린다
            threading.Thread(target=self._heartbeat_loop, daemon=True).start()

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
            threading.Thread(target=self._accept_loop, daemon=True).start()
        for c in self.servers:
            ip = c.get("ip", "127.0.0.1")
            port = int(c.get("port", 9000))
            threading.Thread(target=self._connect_loop, args=(ip, port), daemon=True).start()

    def _accept_loop(self) -> None:
        while self._running:
            try:
                if self._tcp_server is None:
                    break
                conn, addr = self._tcp_server.accept()
                self._register_conn(conn)
                threading.Thread(target=self._recv_loop, args=(conn,), daemon=True).start()
            except OSError:
                break

    def _connect_loop(self, ip: str, port: int) -> None:
        while self._running:
            try:
                conn = socket.create_connection((ip, port), timeout=5.0)
                self._register_conn(conn)
                threading.Thread(target=self._recv_loop, args=(conn,), daemon=True).start()
                return
            except (OSError, ConnectionRefusedError):
                time.sleep(2.0)

    def _register_conn(self, conn: socket.socket) -> None:
        with self._conn_lock:
            self._connections.append(conn)
        # Send discovery info
        self._send(conn, {"type": "discover", "topics": self._local_topics(), "services": self._local_services(), "actions": self._local_actions()})
        # Send latched messages (TRANSIENT_LOCAL history) so late-joining clients get them
        try:
            history = getattr(self.topic_executor, "_history", {})
            for topic, msgs in history.items():
                for msg in msgs:
                    self._send(conn, {"type": "topic", "name": topic, "msg": msg})
        except Exception:
            pass

    def _recv_loop(self, conn: socket.socket) -> None:
        buf = b""
        while self._running:
            try:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data
                while len(buf) >= 4:
                    (length,) = struct.unpack("!I", buf[:4])
                    if len(buf) < 4 + length:
                        break
                    msg = pickle.loads(buf[4:4 + length])
                    buf = buf[4 + length:]
                    self._handle_msg(conn, msg)
            except (OSError, pickle.UnpicklingError, EOFError) as e:
                import logging
                logging.warning("recv_loop error: %s", e)
                break
        with self._conn_lock:
            if conn in self._connections:
                self._connections.remove(conn)
        try:
            conn.close()
        except OSError:
            pass

    def _send(self, conn: socket.socket, msg: dict[str, Any]) -> None:
        try:
            data = pickle.dumps(msg)
            conn.sendall(struct.pack("!I", len(data)) + data)
        except OSError:
            pass

    def _broadcast(self, msg: dict[str, Any], exclude: socket.socket | None = None) -> None:
        with self._conn_lock:
            conns = list(self._connections)
        for conn in conns:
            if conn is exclude:
                continue
            self._send(conn, msg)

    def _handle_msg(self, conn: socket.socket, msg: dict[str, Any]) -> None:
        mtype = msg.get("type")
        if mtype == "heartbeat":
            pass  # 상대가 살아 있음
        elif mtype == "discover":
            # 상대의 토픽/서비스/액션 목록을 받는다
            for t in msg.get("topics", []):
                self._remote_topics.add(t)
                self._register_remote_topic_proxy(t)
            for s in msg.get("services", []):
                self._remote_services.add(s)
            for a in msg.get("actions", []):
                self._remote_actions.add(a)
            # 내 목록도 보낸다
            self._send(conn, {"type": "discover", "topics": self._local_topics(), "services": self._local_services(), "actions": self._local_actions()})
            # latched 메시지도 보낸다 (late subscriber 지원)
            try:
                history = getattr(self.topic_executor, "_history", {})
                for topic, msgs in history.items():
                    for m in msgs:
                        self._send(conn, {"type": "topic", "name": topic, "msg": m})
            except Exception:
                pass
        elif mtype == "topic":
            name = msg.get("name")
            if name is not None:
                # 고빈도 토픽은 10Hz로 스로틀
                throttle_topics = {"scan", "odom", "tf", "tf_static", "imu", "odometry/filtered", "local_costmap/costmap", "global_costmap/costmap"}
                if name in throttle_topics:
                    now = time.monotonic()
                    if not hasattr(self, "_last_recv"):
                        self._last_recv: dict[str, float] = {}
                    last = self._last_recv.get(name, 0.0)
                    if now - last < 0.1:
                        return  # 너무 빠르게 오는 메시지는 건너뛴다
                    self._last_recv[name] = now
                # 네트워크에서 받은 메시지를 로컬 토픽으로 전달 (재브로드캐스트 방지)
                self._receiving = True
                try:
                    self.topic_executor.publish(name, msg.get("msg"), QOS_LATCHED)
                finally:
                    self._receiving = False
        # TODO: service_req, service_res, action_goal, etc.

    def _local_topics(self) -> list[str]:
        # TopicExecutor에 등록된 토픽 목록 (구독자/발행자가 있는 토픽)
        from typing import cast
        return list(cast("dict[str, object]", getattr(self.topic_executor, "_types", {})).keys())

    def _local_services(self) -> list[str]:
        from typing import cast
        return list(cast("dict[str, object]", getattr(self.service_executor, "_servers", {})).keys())

    def _local_actions(self) -> list[str]:
        from typing import cast
        return list(cast("dict[str, object]", getattr(self.action_executor, "_servers", {})).keys())

    def _register_remote_topic_proxy(self, name: str) -> None:
        """원격 토픽을 로컬에 프록시로 등록한다 (구독 시 네트워크에서 가져온다)."""
        # TODO: implement proxy subscription
        pass

    def shutdown(self) -> None:
        self._running = False
        if self._tcp_server is not None:
            try:
                self._tcp_server.close()
            except OSError:
                pass
        with self._conn_lock:
            for conn in self._connections:
                try:
                    conn.close()
                except OSError:
                    pass
            self._connections.clear()
        self.timer_manager.shutdown()
        self.action_executor.shutdown()
        self.service_executor.shutdown()
        self.topic_executor.shutdown()

    # ---- 토픽 (로컬 우선, 없으면 네트워크) ----

    def publish(self, topic: str, msg: Any, qos: QoS = QOS_DEFAULT) -> None:
        self.topic_executor.publish(topic, msg, qos)
        # 네트워크 브로드캐스트 (원격 구독자가 있으면) — 고빈도 토픽은 10Hz로 스로틀
        if self.network_mode and not getattr(self, "_receiving", False):
            now = time.monotonic()
            if not hasattr(self, "_last_broadcast"):
                self._last_broadcast = {}
            last = self._last_broadcast.get(topic, 0.0)
            # 고빈도 토픽 (scan, odom, tf, imu)은 10Hz로 제한
            throttle_topics = {"scan", "odom", "tf", "tf_static", "imu", "odometry/filtered", "local_costmap/costmap"}
            min_interval = 0.1 if topic in throttle_topics else 0.0
            if now - last >= min_interval:
                self._last_broadcast[topic] = now
                self._broadcast({"type": "topic", "name": topic, "msg": msg})

    def subscribe(self, topic: str, msg_type: type, callback: Callable[[Any], None], qos: QoS = QOS_DEFAULT) -> Any:
        # 로컬 구독
        sub = self.topic_executor.subscribe(topic, msg_type, callback, qos)
        return sub

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