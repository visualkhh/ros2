"""cmd_manager — 명령 처리: 외부 요청을 받아 운행 상태를 바꾸고(state_manager에 기록), 바퀴 명령을 고른다(mux).

외부 요청 (누구나 보냄)
  /cmd_manual_control  (String)  MANUAL · NAV_PLAN · SLAM_AUTO_MAPPING (별칭 AUTO→NAV_PLAN, AUTO-MAPPING·MAPPING→SLAM_AUTO_MAPPING)
  /cmd_manual_rl    (Bool)    바퀴 명령이 PC RL 보정을 거치게
  /cmd_manual_estop (Bool)    비상정지
  /cmd_beep         (Empty)   부저 삑 (기본 2000 Hz 150 ms) — 상태 무관, 그대로 전달
  /cmd_beep_tone    (Int32MultiArray [hz, ms])  부저 음 높이·길이 지정 — 상태 무관, 그대로 전달
  /cmd_say          (String)  한/영 문자열 → 자모·알파벳별 주파수 삑으로 변환해 순서대로 재생 (큐, 논블로킹)
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
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Empty, Int32MultiArray, String

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
ZERO = Twist()
MODES = ('MANUAL', 'NAV_PLAN', 'SLAM_AUTO_MAPPING')
ALIASES = {'AUTO': 'NAV_PLAN', 'NAV': 'NAV_PLAN', 'NAV-PLAN': 'NAV_PLAN',
           'SLAM-AUTO-MAPPING': 'SLAM_AUTO_MAPPING', 'AUTO_SLAM_MAPPING': 'SLAM_AUTO_MAPPING',
           'AUTO_MAPPING': 'SLAM_AUTO_MAPPING', 'AUTO-MAPPING': 'SLAM_AUTO_MAPPING', 'MAPPING': 'SLAM_AUTO_MAPPING'}


def text_to_tones(text: str) -> list[tuple[int, int]]:
    """한/영 문자열 → [(Hz, ms)] 삑 순서. tool/cli/op/bus.py와 같은 표 (로봇 쪽 변환본)."""
    tones: list[tuple[int, int]] = []
    for ch in text:
        o = ord(ch)
        if 0xAC00 <= o <= 0xD7A3:  # 한글 음절: 초성 19 · 중성 21 · 종성 28(없음=0)
            j = o - 0xAC00
            tones.append((500 + (j // 588) * 130, 90))
            tones.append((600 + ((j % 588) // 28) * 100, 90))
            if j % 28:
                tones.append((700 + (j % 28) * 80, 90))
        elif 'a' <= (lo := ch.lower()) <= 'z':
            tones.append((500 + (ord(lo) - 97) * 100, 90))
        elif ch.isdigit():
            tones.append((3200 + int(ch) * 100, 90))
        elif ch == ' ':
            tones.append((0, 180))
    return tones


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

        # 부저 — 외부 요청을 그대로 드라이버로 전달 (상태·모드 무관)
        self.pub_beep = self.create_publisher(Empty, 'opencr/beep', 10)
        self.pub_beep_tone = self.create_publisher(Int32MultiArray, 'opencr/beep_tone', 10)
        self.create_subscription(Empty, 'cmd_beep', lambda _m: self.pub_beep.publish(Empty()), 10)
        self.create_subscription(Int32MultiArray, 'cmd_beep_tone', self.pub_beep_tone.publish, 10)
        self._say: deque = deque()   # [(hz, ms)] — tick()이 논블로킹으로 하나씩 재생
        self._say_next = 0.0
        self.create_subscription(String, 'cmd_say', self.on_say, 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_say(self, msg: String) -> None:
        tones = text_to_tones(msg.data.strip())
        if len(self._say) + len(tones) > 200:
            self.get_logger().warning(f'/cmd_say 큐 초과 — 버림 ("{msg.data.strip()}")')
            return
        self._say.extend(tones)

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
        now = time.monotonic()
        if self._say and now >= self._say_next:   # 타이핑삑 재생 (바퀴 mux와 무관)
            hz, ms = self._say.popleft()
            if hz:
                self.pub_beep_tone.publish(Int32MultiArray(data=[hz, ms]))
            self._say_next = now + (ms + 50) / 1000.0
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
