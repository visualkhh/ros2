"""state_manager — 로봇의 현재 논리 상태(도메인 상태)를 알린다. 진단(전류·온도)은 여기 없음 (/diagnostics).

  제어 상태 — 바퀴를 누가 움직이나. 바꾸는 판단은 cmd_manager (외부 요청 /cmd_manual_* → cmd_manager → /state_set_*)
    sub /state_set_control · /state_set_rl · /state_set_estop
    pub /state_control (String)  MANUAL · NAV_PLAN · SLAM_AUTO_MAPPING
        /state_rl      (Bool)    바퀴 명령 경로: false 직결 · true PC RL 경유
        /state_estop   (Bool)    비상정지
  운행 상태 — 위치를 어떻게 아나. 떠 있는 노드로 판단 (2초마다)
    pub /state_operation (String)  SLAM (cartographer_node) · LOCALIZATION (amcl·map_server) · IDLE
        /state_map       (String)  적용된 지도 이름 (map_server의 yaml_filename 폴더명) · SLAM이면 '(SLAM 작성 중)' · 없으면 ''
  전부 latched (나중에 붙은 쪽도 현재 값을 받음).
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
SLAM_MAP = '(SLAM 작성 중)'


class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        self.values = {'control': 'MANUAL', 'rl': bool(self.declare_parameter('rl', False).value), 'estop': False,
                       'operation': 'IDLE', 'map': ''}
        self.pubs = {k: self.create_publisher(String if isinstance(v, str) else Bool, f'state_{k}', LATCHED)
                     for k, v in self.values.items()}
        self.create_subscription(String, 'state_set_control', lambda m: self.set('control', m.data), 10)
        self.create_subscription(Bool, 'state_set_rl', lambda m: self.set('rl', m.data), 10)
        self.create_subscription(Bool, 'state_set_estop', lambda m: self.set('estop', m.data), 10)
        self.map_params = AsyncParameterClient(self, 'map_server')
        for key in self.values:
            self.publish(key)
        self.create_timer(2.0, self.watch_operation)

    def set(self, key: str, value) -> None:
        if self.values[key] != value:
            self.get_logger().info(f'{key}: {self.values[key]!r} -> {value!r}')
            self.values[key] = value
            self.publish(key)

    def publish(self, key: str) -> None:
        value = self.values[key]
        self.pubs[key].publish(String(data=value) if isinstance(value, str) else Bool(data=value))

    def watch_operation(self) -> None:
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        if 'cartographer_node' in names:
            self.set('operation', 'SLAM')
            self.set('map', SLAM_MAP)
        elif 'amcl' in names or 'map_server' in names:
            self.set('operation', 'LOCALIZATION')
            if self.map_params.services_are_ready():
                self.map_params.get_parameters(['yaml_filename']).add_done_callback(self.on_map_param)
        else:
            self.set('operation', 'IDLE')
            self.set('map', '')

    def on_map_param(self, fut) -> None:
        try:
            path = fut.result().values[0].string_value
        except Exception:
            return
        if path and self.values['operation'] == 'LOCALIZATION':
            self.set('map', os.path.basename(os.path.dirname(path)) or os.path.basename(path))


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
