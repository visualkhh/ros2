"""cmd_manager — wheel command mux. Picks the source by /state_* and routes direct or through PC RL.

  sub  /cmd_manual_vel (Twist)   teleop / tool drive          used when /state_mode = MANUAL
  sub  /cmd_vel        (Twist)   Nav2 output                  used when /state_mode = AUTO
  sub  /state_mode · /state_rl · /state_estop                  (state_manager, latched)
  rl off:  picked -> /cmd_vel_base -> opencr_driver
  rl on:   picked -> /cmd_vel_rl_in -> [PC rl_corrector] -> /cmd_vel_rl_out -> /cmd_vel_base

Safety: E-stop, stale source (> source_timeout) or stale RL output (> rl_timeout) -> zero.
Publishes at `rate` Hz so the driver always has a fresh command (it also stops on its own timeout).
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
ZERO = Twist()


class CmdManager(Node):
    def __init__(self):
        super().__init__('cmd_manager')
        p = self.declare_parameter
        self.source_timeout = p('source_timeout', 0.5).value
        self.rl_timeout = p('rl_timeout', 0.3).value     # WiFi round trip Pi -> PC -> Pi
        rate = p('rate', 20.0).value
        self.mode, self.rl, self.estop = 'MANUAL', False, False
        self.last = {}   # topic -> (Twist, monotonic time)

        for topic in ('cmd_manual_vel', 'cmd_vel', 'cmd_vel_rl_out'):
            self.create_subscription(Twist, topic, lambda m, t=topic: self.last.__setitem__(t, (m, time.monotonic())), 10)
        self.create_subscription(String, 'state_mode', lambda m: setattr(self, 'mode', m.data), LATCHED)
        self.create_subscription(Bool, 'state_rl', lambda m: setattr(self, 'rl', m.data), LATCHED)
        self.create_subscription(Bool, 'state_estop', lambda m: setattr(self, 'estop', m.data), LATCHED)
        self.pub_base = self.create_publisher(Twist, 'cmd_vel_base', 10)
        self.pub_rl_in = self.create_publisher(Twist, 'cmd_vel_rl_in', 10)
        self.create_timer(1.0 / rate, self.tick)

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
