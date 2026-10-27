"""Movement correction RL node.

cmd_vel_in -> [optional ONNX policy] -> clip -> cmd_vel_out

Before/After toggling is done with remap only, no code change:
  Before: teleop/nav2 -> cmd_vel (direct to base)
  After:  teleop/nav2 -> cmd_vel_in -> this node -> cmd_vel_out -> cmd_vel (remap out to cmd_vel)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RlCorrector(Node):
    def __init__(self):
        super().__init__('rl_corrector')
        self.declare_parameter('enable', False)
        self.declare_parameter('model_path', '')
        # TurtleBot3 Burger limits
        self.declare_parameter('max_lin', 0.22)
        self.declare_parameter('max_ang', 2.84)
        self.session = None
        self.in_name = None
        self._load_model()
        self.sub = self.create_subscription(Twist, 'cmd_vel_in', self.cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

    def _load_model(self):
        path = self.get_parameter('model_path').value
        if not path:
            self.get_logger().info('No model_path: passthrough mode')
            return
        try:
            import onnxruntime as ort
            self.session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
            self.in_name = self.session.get_inputs()[0].name
            self.get_logger().info(f'ONNX loaded: {path}')
        except Exception as e:  # keep node alive as passthrough
            self.get_logger().warn(f'ONNX load failed ({e}): passthrough')

    def _infer_delta(self, lin, ang):
        x = np.array([[lin, ang]], dtype=np.float32)
        out = self.session.run(None, {self.in_name: x})[0]
        return float(out[0][0]), float(out[0][1])

    def cb(self, msg):
        lin, ang = msg.linear.x, msg.angular.z
        if self.get_parameter('enable').value and self.session is not None:
            dlin, dang = self._infer_delta(lin, ang)
            lin, ang = lin + dlin, ang + dang
        max_lin = self.get_parameter('max_lin').value
        max_ang = self.get_parameter('max_ang').value
        out = Twist()
        out.linear.x = max(-max_lin, min(max_lin, lin))
        out.angular.z = max(-max_ang, min(max_ang, ang))
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RlCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
