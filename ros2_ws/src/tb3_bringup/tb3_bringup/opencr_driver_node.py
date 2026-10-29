"""OpenCR(tb3_wheels firmware) driver: cmd_vel -> 'C,l,r' serial, 'S,...' -> odom + TF + joint_states.

Protocol (firmware/opencr/README.md):
  -> C,<left rad/s>,<right rad/s>   T,0|1   R[,l,r]   B[,hz,ms]
  <- S,<l_pos>,<l_vel>,<r_pos>,<r_vel>,<uptime ms>,<l_cur A>,<r_cur A>,<l_temp C>,<r_temp C>,<battery V>
  <- E,<l_err>,<r_err>              (hardware error bitmask, -1 = no response)
  <- I,<gx,gy,gz rad/s>,<ax,ay,az m/s^2>,<qw,qx,qy,qz>   OpenCR on-board IMU -> imu (frame imu_link)

Topics:
  sub  <cmd_topic> (Twist, default cmd_vel_base)   final wheel command from cmd_manager (mux)
  sub  opencr/reboot (Empty)                   -> R (clears DYNAMIXEL hardware error)
  sub  opencr/beep (Empty)                     -> B (OpenCR buzzer, 2000 Hz 150 ms)
  sub  opencr/beep_tone (Int32MultiArray)     -> B,<hz>,<ms> (100~8000 Hz, 10~2000 ms)
  pub  odom, TF odom->base_footprint (REP-105), joint_states, diagnostics, battery_state, imu
The firmware stops the wheels by itself if no C arrives for 500 ms, so a dead PC/RL link stops the robot.
"""
import math
import os
import select
import termios
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Empty, Int32MultiArray
from tf2_ros import TransformBroadcaster

ERR_BITS = {1: 'input voltage', 4: 'overheating', 8: 'motor encoder', 16: 'electrical shock', 32: 'overload'}


def err_text(code: int) -> str:
    if code == 0:
        return 'ok'
    if code < 0:
        return 'no response'
    return '+'.join(name for bit, name in ERR_BITS.items() if code & bit) or str(code)


def wheel_speeds(v: float, w: float, r: float, sep: float, max_wheel: float) -> tuple[float, float]:
    """Twist -> (left, right) wheel rad/s. If one wheel would exceed max_wheel, scale BOTH by the same ratio
    so the arc keeps its shape (firmware clips each wheel alone -> a fast arc turns into a pivot on the inner wheel)."""
    wl = (v - w * sep / 2.0) / r
    wr = (v + w * sep / 2.0) / r
    peak = max(abs(wl), abs(wr))
    if max_wheel > 0.0 and peak > max_wheel:
        wl, wr = wl * max_wheel / peak, wr * max_wheel / peak
    return wl, wr


class OpencrDriver(Node):
    def __init__(self):
        super().__init__('opencr_driver')
        p = self.declare_parameter
        self.port = p('port', '/dev/opencr').value
        self.baud = p('baud', 115200).value
        self.r = p('wheel_radius', 0.033).value            # TB3 Waffle Pi
        self.sep = p('wheel_separation', 0.287).value      # TB3 Waffle Pi
        self.max_wheel = p('max_wheel_speed', 8.0).value     # [rad/s] = firmware VEL_LIMIT_RAD_S (0.26 m/s)
        # Mounting direction per wheel (this robot: both -1, config/robot.yaml).
        # Check: ./run-cli drive forward → both wheels must roll forward.
        self.dir_l = p('left_dir', -1.0).value
        self.dir_r = p('right_dir', -1.0).value
        self.cmd_rate = p('cmd_rate', 20.0).value          # C resend rate (watchdog is 500 ms)
        self.cmd_timeout = p('cmd_timeout', 0.5).value     # no cmd_vel for this long -> send 0
        self.publish_tf = p('publish_tf', True).value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        cmd_topic = p('cmd_topic', 'cmd_vel_base').value

        self.fd = None
        self.buf = b''
        self.cmd = (0.0, 0.0)
        self.last_cmd = 0.0
        self.prev_pos = None
        self.x = self.y = self.yaw = 0.0
        self.last_s = None          # parsed S fields
        self.last_s_time = 0.0
        self.errors = (0, 0)
        self.jumps = 0

        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(Empty, 'opencr/reboot', lambda _m: self.write('R'), 10)
        self.create_subscription(Empty, 'opencr/beep', lambda _m: self.write('B'), 10)
        self.create_subscription(Int32MultiArray, 'opencr/beep_tone', self.on_beep_tone, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.bat_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.02, self.read_serial)   # S 20Hz 스트림 폴링 (200Hz는 Pi CPU 과다)
        self.create_timer(1.0 / self.cmd_rate, self.send_cmd)
        self.create_timer(1.0, self.publish_diag)
        self.get_logger().info(f'cmd: {cmd_topic} -> {self.port} · r={self.r} sep={self.sep} '
                               f'dir L{self.dir_l:+.0f} R{self.dir_r:+.0f}')

    # ------------------------------------------------------------ serial
    def open(self) -> bool:
        if self.fd is not None:
            return True
        try:
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            self.get_logger().warning(f'open {self.port}: {exc}', throttle_duration_sec=5.0)
            return False
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0                       # raw: iflag, oflag, lflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        speed = getattr(termios, f'B{self.baud}')
        attrs[4] = attrs[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self.fd, self.buf, self.prev_pos = fd, b'', None
        self.get_logger().info(f'opened {self.port}')
        return True

    def close(self, why: str) -> None:
        if self.fd is not None:
            self.get_logger().error(f'{self.port}: {why} — reconnecting')
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None

    def write(self, line: str) -> None:
        if not self.open():
            return
        try:
            os.write(self.fd, (line + '\n').encode())
        except OSError as exc:
            self.close(f'write: {exc}')

    def read_serial(self) -> None:
        if not self.open():
            return
        try:
            if not select.select([self.fd], [], [], 0)[0]:
                return
            data = os.read(self.fd, 4096)
        except OSError as exc:
            self.close(f'read: {exc}')
            return
        if not data:
            return
        self.buf += data
        while b'\n' in self.buf:
            raw, self.buf = self.buf.split(b'\n', 1)
            self.handle_line(raw.decode(errors='replace').strip())
        if len(self.buf) > 1024:
            self.buf = b''

    # ------------------------------------------------------------ commands
    def on_cmd(self, msg: Twist) -> None:
        self.cmd = wheel_speeds(msg.linear.x, msg.angular.z, self.r, self.sep, self.max_wheel)
        self.last_cmd = time.monotonic()

    def on_beep_tone(self, msg: Int32MultiArray) -> None:
        hz = int(msg.data[0]) if len(msg.data) > 0 else 2000
        ms = int(msg.data[1]) if len(msg.data) > 1 else 150
        hz = max(100, min(8000, hz))
        ms = max(10, min(2000, ms))
        self.write(f'B,{hz},{ms}')

    def send_cmd(self) -> None:
        wl, wr = self.cmd
        if time.monotonic() - self.last_cmd > self.cmd_timeout:
            wl = wr = 0.0
            self.cmd = (0.0, 0.0)
        self.write(f'C,{wl * self.dir_l:.4f},{wr * self.dir_r:.4f}')

    # ------------------------------------------------------------ status
    def handle_line(self, line: str) -> None:
        f = line.split(',')
        if f[0] == 'S' and len(f) >= 6:
            try:
                vals = [float(x) for x in f[1:]]
            except ValueError:
                return
            self.on_status(vals)
        elif f[0] == 'I' and len(f) >= 11:
            try:
                self.on_imu([float(x) for x in f[1:11]])
            except ValueError:
                return
        elif f[0] == 'E' and len(f) >= 3:
            try:
                el, er = int(f[1]), int(f[2])
            except ValueError:
                return
            if (el, er) != self.errors:
                log = self.get_logger().info if (el, er) == (0, 0) else self.get_logger().error
                log(f'motor error: left {err_text(el)} · right {err_text(er)}'
                    + ('' if (el, er) == (0, 0) else ' → ros2 topic pub --once /opencr/reboot std_msgs/msg/Empty'))
            self.errors = (el, er)

    def on_imu(self, v: list) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = v[0:3]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = v[3:6]
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = v[6:10]
        self.imu_pub.publish(msg)

    def on_status(self, v: list) -> None:
        now = self.get_clock().now()
        pl, vl, pr, vr = v[0] * self.dir_l, v[1] * self.dir_l, v[2] * self.dir_r, v[3] * self.dir_r
        self.last_s, self.last_s_time = v, time.monotonic()
        if self.prev_pos is not None:
            dl, dr = pl - self.prev_pos[0], pr - self.prev_pos[1]
            # Reboot (R) restarts multi-turn position within one turn -> ignore that jump.
            if abs(dl) > 1.0 or abs(dr) > 1.0:
                self.jumps += 1
                self.get_logger().warning(f'wheel position jump ({dl:+.2f}, {dr:+.2f}) rad ignored (reboot?)')
            else:
                ds = self.r * (dl + dr) / 2.0
                dth = self.r * (dr - dl) / self.sep
                self.x += ds * math.cos(self.yaw + dth / 2.0)
                self.y += ds * math.sin(self.yaw + dth / 2.0)
                self.yaw += dth
        self.prev_pos = (pl, pr)

        v_lin = self.r * (vl + vr) / 2.0
        v_ang = self.r * (vr - vl) / self.sep
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_lin
        odom.twist.twist.angular.z = v_ang
        for i, c in ((0, 0.01), (7, 0.01), (35, 0.03)):
            odom.pose.covariance[i] = c
            odom.twist.covariance[i] = c
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)

        js = JointState()
        js.header.stamp = odom.header.stamp
        js.name = ['wheel_left_joint', 'wheel_right_joint']
        js.position = [pl, pr]
        js.velocity = [vl, vr]
        if len(v) >= 7:
            js.effort = [v[5], v[6]]  # current [A]
        self.js_pub.publish(js)

        if len(v) >= 10:
            # 3S LiPo: full 12.6 V, empty ~9.6 V
            volt = v[9]
            bat = BatteryState()
            bat.header.stamp = odom.header.stamp
            bat.present = True
            bat.voltage = volt
            bat.percentage = max(0.0, min(1.0, (volt - 9.6) / 3.0))
            bat.design_capacity = 1.8  # Ah (TB3 LB-040 class)
            self.bat_pub.publish(bat)

    def publish_diag(self) -> None:
        st = DiagnosticStatus()
        st.name = 'opencr'
        st.hardware_id = self.port
        age = time.monotonic() - self.last_s_time if self.last_s else None
        if self.fd is None:
            st.level, st.message = DiagnosticStatus.ERROR, f'{self.port} not open'
        elif age is None or age > 1.0:
            st.level, st.message = DiagnosticStatus.ERROR, 'no S line (firmware?)'
        elif self.errors != (0, 0):
            st.level = DiagnosticStatus.ERROR
            st.message = f'motor error L:{err_text(self.errors[0])} R:{err_text(self.errors[1])}'
        else:
            st.level, st.message = DiagnosticStatus.OK, 'ok'
        if self.last_s and len(self.last_s) >= 9:
            v = self.last_s
            st.values = [KeyValue(key=k, value=f'{x:g}') for k, x in (
                ('left_current_A', v[5]), ('right_current_A', v[6]),
                ('left_temp_C', v[7]), ('right_temp_C', v[8]), ('uptime_ms', v[4]))]
            if len(v) >= 10:
                st.values.append(KeyValue(key='battery_V', value=f'{v[9]:g}'))
                st.values.append(KeyValue(key='battery_pct',
                                          value=f'{max(0.0, min(1.0, (v[9] - 9.6) / 3.0)) * 100:g}'))
        st.values.append(KeyValue(key='position_jumps', value=str(self.jumps)))
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)

    def shutdown(self) -> None:
        for _ in range(3):
            self.write('C,0,0')


def main(args=None):
    rclpy.init(args=args)
    node = OpencrDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
