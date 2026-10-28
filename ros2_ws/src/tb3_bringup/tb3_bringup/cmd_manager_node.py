"""cmd_manager — 명령 처리: 외부 요청을 받아 운행 상태를 바꾸고(state_manager에 기록), 바퀴 명령을 고른다(mux).

외부 요청 (누구나 보냄)
  /cmd_manual_control  (String)  MANUAL · NAV_PLAN · SLAM_AUTO_MAPPING (별칭 AUTO→NAV_PLAN, AUTO-MAPPING·MAPPING→SLAM_AUTO_MAPPING)
  /cmd_manual_rl    (Bool)    바퀴 명령이 PC RL 보정을 거치게
  /cmd_manual_estop (Bool)    비상정지
  /goal_pose        (PoseStamped)  RViz Nav2 Goal → NAV_PLAN
  → 검사 후 /state_set_* → state_manager가 /state_* (latched)로 알림. 이 노드도 /state_*를 따라 동작.

바퀴 명령
  /cmd_manual_vel (Twist)  수동 (텔레오퍼·tool drive) — /state_control = MANUAL일 때만
  /cmd_vel        (Twist)  Nav2 — NAV_PLAN · SLAM_AUTO_MAPPING일 때만
  rl off:  고른 것 -> /cmd_vel_base -> opencr_driver
  rl on:   고른 것 -> /cmd_vel_rl_in -> [PC rl_corrector] -> /cmd_vel_rl_out -> /cmd_vel_base

안전: 비상정지 · 고른 입력이 source_timeout 넘게 끊김 · RL on인데 보정값이 rl_timeout 넘게 끊김 → 0.
rate Hz로 계속 보내서 드라이버는 늘 새 명령을 받는다 (드라이버·펌웨어도 각자 멈춤).
"""
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
ZERO = Twist()
MODES = ('MANUAL', 'NAV_PLAN', 'SLAM_AUTO_MAPPING')
ALIASES = {'AUTO': 'NAV_PLAN', 'NAV': 'NAV_PLAN', 'NAV-PLAN': 'NAV_PLAN',
           'SLAM-AUTO-MAPPING': 'SLAM_AUTO_MAPPING', 'AUTO_SLAM_MAPPING': 'SLAM_AUTO_MAPPING',
           'AUTO_MAPPING': 'SLAM_AUTO_MAPPING', 'AUTO-MAPPING': 'SLAM_AUTO_MAPPING', 'MAPPING': 'SLAM_AUTO_MAPPING'}


class CmdManager(Node):
    def __init__(self):
        super().__init__('cmd_manager')
        p = self.declare_parameter
        self.source_timeout = p('source_timeout', 0.5).value
        self.rl_timeout = p('rl_timeout', 0.3).value     # WiFi 왕복 Pi -> PC -> Pi
        rate = p('rate', 20.0).value
        self.mode, self.rl, self.estop = 'MANUAL', False, False
        self.last = {}   # topic -> (Twist, monotonic time)

        # 외부 요청 → 검사 → state_manager에 기록
        self.set_mode = self.create_publisher(String, 'state_set_control', 10)
        self.set_rl = self.create_publisher(Bool, 'state_set_rl', 10)
        self.set_estop = self.create_publisher(Bool, 'state_set_estop', 10)
        self.create_subscription(String, 'cmd_manual_control', self.on_mode_request, 10)
        self.create_subscription(Bool, 'cmd_manual_rl', lambda m: self.set_rl.publish(m), 10)
        self.create_subscription(Bool, 'cmd_manual_estop', self.on_estop_request, 10)
        self.create_subscription(PoseStamped, 'goal_pose', lambda _m: self.request_control('NAV_PLAN', 'RViz Nav2 Goal'), 10)

        # 현재 상태 (state_manager, latched)
        self.create_subscription(String, 'state_control', lambda m: setattr(self, 'mode', m.data), LATCHED)
        self.create_subscription(Bool, 'state_rl', lambda m: setattr(self, 'rl', m.data), LATCHED)
        self.create_subscription(Bool, 'state_estop', lambda m: setattr(self, 'estop', m.data), LATCHED)

        # 바퀴 명령
        for topic in ('cmd_manual_vel', 'cmd_vel', 'cmd_vel_rl_out'):
            self.create_subscription(Twist, topic, lambda m, t=topic: self.last.__setitem__(t, (m, time.monotonic())), 10)
        self.pub_base = self.create_publisher(Twist, 'cmd_vel_base', 10)
        self.pub_rl_in = self.create_publisher(Twist, 'cmd_vel_rl_in', 10)
        self.create_timer(1.0 / rate, self.tick)

    # ------------------------------------------------------------ 상태 변경 요청
    def on_mode_request(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        self.request_control(ALIASES.get(mode, mode), '요청')

    def request_control(self, mode: str, why: str) -> None:
        if mode not in MODES:
            self.get_logger().warning(f'모르는 모드 {mode!r} — {"/".join(MODES)}')
            return
        if mode != self.mode:
            self.get_logger().info(f'제어 상태 {self.mode} -> {mode} ({why})')
            self.last.pop('cmd_manual_vel', None)   # 이전 모드에서 온 수동 명령이 새 모드로 새지 않게
        self.set_mode.publish(String(data=mode))

    def on_estop_request(self, msg: Bool) -> None:
        if msg.data:
            self.pub_base.publish(ZERO)   # 상태 왕복을 기다리지 않고 바로 정지
        self.set_estop.publish(msg)

    # ------------------------------------------------------------ mux
    def fresh(self, topic: str, timeout: float) -> Twist:
        msg, t = self.last.get(topic, (ZERO, 0.0))
        return msg if time.monotonic() - t <= timeout else ZERO

    def tick(self) -> None:
        if self.estop:
            self.pub_base.publish(ZERO)
            return
        picked = self.fresh('cmd_manual_vel' if self.mode == 'MANUAL' else 'cmd_vel', self.source_timeout)
        if not self.rl:
            self.pub_base.publish(picked)
            return
        self.pub_rl_in.publish(picked)
        self.pub_base.publish(self.fresh('cmd_vel_rl_out', self.rl_timeout))


def main(args=None):
    rclpy.init(args=args)
    node = CmdManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_base.publish(ZERO)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
