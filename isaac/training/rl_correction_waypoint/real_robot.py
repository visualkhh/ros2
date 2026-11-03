"""RealRobot — 이 로봇의 실기기 (ROS 2). PC에서 Pi의 base · cmd_manager에 붙는다. rclpy.init() 뒤에 만든다.

  write  /cmd_manual_vel (Twist) — cmd_manager가 MANUAL일 때 모터로
  read   /odom · /imu · /joint_states (wheel_left_joint · wheel_right_joint) · /cmd_vel_base (실제로 나간 명령)
  state  /state_control (MANUAL 등)
"""
from __future__ import annotations

import math
import time

from robots.robot_interface import RobotInterface


def _sec(ts) -> float:
    if ts is None:
        return float("nan")
    if hasattr(ts, "nanoseconds"):   # rclpy Time
        return ts.nanoseconds / 1e9
    return float(ts.sec) + float(ts.nanosec) * 1e-9


class RealRobot(RobotInterface):
    def __init__(self, rate: float = 20.0):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String

        super().__init__(1.0 / rate)
        self.rclpy, self.Twist, self.rate = rclpy, Twist, rate
        self.node = node = rclpy.create_node("real_robot")
        self.s = s = {"state": None, "odom": None, "imu_w": 0.0, "imu_stamp": None, "joint_stamp": None,
                      "wl": 0.0, "wr": 0.0, "pl": 0.0, "pr": 0.0, "out": (0.0, 0.0)}

        def on_joint(m):
            s["joint_stamp"] = m.header.stamp
            for name, pos, vel in zip(m.name, m.position, m.velocity):
                if name == "wheel_left_joint":
                    s["pl"], s["wl"] = pos, vel
                elif name == "wheel_right_joint":
                    s["pr"], s["wr"] = pos, vel

        def on_imu(m):
            s["imu_w"], s["imu_stamp"] = m.angular_velocity.z, m.header.stamp

        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "/state_control", lambda m: s.update(state=m.data), latched)
        node.create_subscription(Odometry, "/odom", lambda m: s.update(odom=m), 10)
        node.create_subscription(Imu, "/imu", on_imu, 10)
        node.create_subscription(JointState, "/joint_states", on_joint, 10)
        node.create_subscription(Twist, "/cmd_vel_base", lambda m: s.update(out=(m.linear.x, m.angular.z)), 10)
        self.pub = node.create_publisher(Twist, "/cmd_manual_vel", 10)

    def write(self, v, w):
        msg = self.Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(3):
            self.write(0.0, 0.0)
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait(self, key, sec):
        """self.s[key] 메시지가 올 때까지 최대 sec초 (state · odom 등)."""
        end = time.monotonic() + sec
        while self.s[key] is None and time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def start(self):
        self.t0 = time.monotonic()   # pace 기준 (벽시계)

    def pace(self, k):
        nxt = self.t0 + (k + 1) / self.rate
        while time.monotonic() < nxt:
            self.rclpy.spin_once(self.node, timeout_sec=min(0.01, max(0.0, nxt - time.monotonic())))

    def read(self):
        s, o = self.s, self.s["odom"]
        pp, q = o.pose.pose.position, o.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {"lin_out": s["out"][0], "ang_out": s["out"][1], "odom_x": pp.x, "odom_y": pp.y, "odom_yaw": yaw,
                "v_enc": o.twist.twist.linear.x, "w_enc": o.twist.twist.angular.z, "w_gyro": s["imu_w"],
                "wheel_l_vel": s["wl"], "wheel_r_vel": s["wr"], "wheel_l_pos": s["pl"], "wheel_r_pos": s["pr"],
                "t_ros": _sec(self.node.get_clock().now()), "t_odom": _sec(o.header.stamp),
                "t_imu": _sec(s["imu_stamp"]), "t_joint": _sec(s["joint_stamp"])}

    def close(self):
        self.node.destroy_node()
