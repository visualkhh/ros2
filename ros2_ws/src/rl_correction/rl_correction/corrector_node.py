"""Movement correction RL node (task-spec v1).

obs  [v_cmd, w_cmd, v_odom, w_odom] -> ONNX -> [dV, dW] -> clip -> cmd_vel_out
No model (or dim mismatch): transparent passthrough.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RlCorrector(Node):
    def __init__(self):
        super().__init__('rl_corrector')
        self.declare_parameter('enable', False)
        self.declare_parameter('model_path', '')
        # TurtleBot3 Waffle Pi limits
        self.declare_parameter('max_lin', 0.22)
        self.declare_parameter('max_ang', 2.84)
        self.session = None
        self.in_name = None
        self.obs_dim = 0
        self.v_odom = 0.0
        self.w_odom = 0.0
        self._load_model()
        self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
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
            self.obs_dim = int(self.session.get_inputs()[0].shape[1])
            self.get_logger().info(f'ONNX loaded: {path} (obs_dim={self.obs_dim})')
        except Exception as e:  # keep node alive as passthrough
            self.get_logger().warning(f'ONNX load failed ({e}): passthrough')

    def cb_odom(self, msg):
        self.v_odom = msg.twist.twist.linear.x
        self.w_odom = msg.twist.twist.angular.z

    def _infer_delta(self, obs):
        x = np.array([obs], dtype=np.float32)
        out = self.session.run(None, {self.in_name: x})[0]
        return float(out[0][0]), float(out[0][1])

    def cb(self, msg):
        lin, ang = msg.linear.x, msg.angular.z
        if self.get_parameter('enable').value and self.session is not None:
            obs = [lin, ang, self.v_odom, self.w_odom]
            if len(obs) == self.obs_dim:
                dlin, dang = self._infer_delta(obs)
                lin, ang = lin + dlin, ang + dang
            else:
                self.get_logger().warning(
                    f'obs dim {len(obs)} != model {self.obs_dim}: passthrough', throttle_duration_sec=5.0)
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
