"""Mock differential-drive base for PC test (no TurtleBot hardware needed).

Subscribes cmd_vel, integrates pose, publishes odom + TF odom->base_footprint->base_link (REP-105,
same tree as opencr_driver + URDF).
Real robot: replace this node with the TurtleBot3 driver (OpenCR/Dynamixel),
topics stay the same.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class MockBase(Node):
    def __init__(self):
        super().__init__('mock_base')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last = self.get_clock().now()
        self.create_subscription(Twist, 'cmd_vel', self.cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        # PC 시험엔 URDF가 없으므로 base_footprint->base_link(바퀴축 +0.010)도 여기서 낸다
        self.static_tf = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'base_footprint'
        st.child_frame_id = 'base_link'
        st.transform.translation.z = 0.010
        st.transform.rotation.w = 1.0
        self.static_tf.sendTransform(st)
        self.create_timer(0.05, self.tick)  # 20 Hz

    def cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds / 1e9
        self.last = now
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = math.sin(self.yaw / 2)
        qw = math.cos(self.yaw / 2)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MockBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
