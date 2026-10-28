"""state_manager — owner of the robot's logical (domain) state. Diagnostics (current, temp) are NOT here.

  sub  /cmd_manual_mode   (String  MANUAL | AUTO)   who drives: teleop or Nav2
  sub  /cmd_manual_rl     (Bool)                    route wheel commands through PC RL correction
  sub  /cmd_manual_estop  (Bool)                    emergency stop
  sub  /goal_pose         (PoseStamped)             RViz Nav2 Goal -> mode AUTO (last action decides; tool teleop -> MANUAL)
  pub  /state_mode · /state_rl · /state_estop      latched (transient local): late joiners get the current value

cmd_manager follows these to pick and route the wheel command (see ARCH.md).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
MODES = ('MANUAL', 'AUTO')


class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        self.mode = self.declare_parameter('mode', 'MANUAL').value
        self.rl = self.declare_parameter('rl', False).value
        self.estop = False
        self.pub_mode = self.create_publisher(String, 'state_mode', LATCHED)
        self.pub_rl = self.create_publisher(Bool, 'state_rl', LATCHED)
        self.pub_estop = self.create_publisher(Bool, 'state_estop', LATCHED)
        self.create_subscription(String, 'cmd_manual_mode', self.on_mode, 10)
        self.create_subscription(Bool, 'cmd_manual_rl', lambda m: self.set('rl', m.data), 10)
        self.create_subscription(Bool, 'cmd_manual_estop', lambda m: self.set('estop', m.data), 10)
        self.create_subscription(PoseStamped, 'goal_pose', lambda _m: self.set('mode', 'AUTO'), 10)
        self.publish()

    def on_mode(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        if mode not in MODES:
            self.get_logger().warning(f'unknown mode {msg.data!r} (use {"/".join(MODES)})')
            return
        self.set('mode', mode)

    def set(self, key: str, value) -> None:
        if getattr(self, key) != value:
            self.get_logger().info(f'{key}: {getattr(self, key)} -> {value}')
            setattr(self, key, value)
        self.publish()

    def publish(self) -> None:
        self.pub_mode.publish(String(data=self.mode))
        self.pub_rl.publish(Bool(data=self.rl))
        self.pub_estop.publish(Bool(data=self.estop))


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
