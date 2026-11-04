"""RealRobot — 이 로봇의 실기기 (ROS 2). PC에서 Pi의 base · cmd_manager에 붙는다. rclpy.init() 뒤에 만든다.

  write  /cmd_manual_vel (Twist) — cmd_manager가 MANUAL일 때 모터로
  기록   /odom · /imu · /joint_states (wheel_left_joint · wheel_right_joint) · /cmd_vel_base (실제로 나간 명령) —
         측정 중 스텝마다 pace() 끝에 최신 값을 한 줄 쌓는다
  state  /state_control (MANUAL 등)
  onnx   PC rl_corrector (route rl일 때 명령이 거쳐 감): onnx 경로를 주면 그 모델 + 적용(B), 안 주면 바이패스(A · calibration)
         — /rl_corrector/set_parameters(model_path) · /rl_corrector/toggle(SetBool). corrector가 없으면(route direct) 그대로 진행
"""
from __future__ import annotations

import math
import os
import time

from robots.robot_interface import RobotInterface


def _sec(ts) -> float:
    if ts is None:
        return float("nan")
    if hasattr(ts, "nanoseconds"):   # rclpy Time
        return ts.nanoseconds / 1e9
    return float(ts.sec) + float(ts.nanosec) * 1e-9


class RealRobot(RobotInterface):
    def __init__(self, rate: float = 20.0, onnx: str | None = None):
        import rclpy

        if not rclpy.ok():
            rclpy.init()
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String

        super().__init__(1.0 / rate)
        self.rclpy, self.Twist, self.rate, self.onnx = rclpy, Twist, rate, onnx
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
        self.sent = (float(v), float(w))
        msg = self.Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def ready(self):
        """MANUAL이고 /odom이 와야 움직인다. /imu · /joint_states가 없으면 경고만."""
        self.wait("state", 5.0)
        if self.s["state"] != "MANUAL":
            return f"MANUAL 아님 (state={self.s['state']}) — Pi에서 ./run-cli mode manual 먼저"
        self.wait("odom", 3.0)
        if self.s["odom"] is None:
            return "/odom 없음 — base/드라이버 확인"
        self.wait("imu_stamp", 1.0)
        self.wait("joint_stamp", 1.0)
        if self.s["imu_stamp"] is None:
            print("[real] 경고: /imu 없음 — w_gyro가 0")
        if self.s["joint_stamp"] is None:
            print("[real] 경고: /joint_states 없음 — 바퀴 raw가 0")
        return None

    def _call(self, srv_type, name: str, req, timeout: float = 3.0):
        cli = self.node.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def _set_correction(self) -> str | None:
        """onnx 있으면 그 모델로 corrector 적용(B), 없으면 바이패스(A). 결과를 기록 칸 rl_onnx로."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        from std_srvs.srv import SetBool

        if self.onnx:
            val = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=self.onnx)
            r = self._call(SetParameters, "/rl_corrector/set_parameters",
                           SetParameters.Request(parameters=[Parameter(name="model_path", value=val)]))
            if r is None:
                return "rl_corrector 없음 — PC에서 ./rl-only.sh 띄우고 Pi route rl (--onnx는 corrector가 있어야)"
        r = self._call(SetBool, "/rl_corrector/toggle", SetBool.Request(data=bool(self.onnx)))
        if r is None:
            self.tag = "direct"
            print("[real] rl_corrector 없음 — 보정 없이 (route direct면 정상)")
            return None
        if self.onnx and "model=loaded" not in r.message:
            return f"ONNX 로드 실패: {self.onnx} ({r.message}) — rl-only 로그 확인"
        self.tag = os.path.basename(self.onnx) if self.onnx else "bypass"
        print(f"[real] rl_corrector: {r.message}")
        return None

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
        why = self._set_correction()   # 측정 직전에만 corrector를 바꾼다 (--check-only는 안 건드림)
        if why:
            raise SystemExit(f"[real] 거부: {why}")
        super().start()
        self.t0 = time.monotonic()   # pace 기준 (벽시계)

    def pace(self, k):
        """k번째 스텝 끝까지 ROS 메시지를 받으며 기다린 뒤, 측정 중이면 그때의 최신 값을 쌓는다."""
        nxt = self.t0 + (k + 1) / self.rate
        while time.monotonic() < nxt:
            self.rclpy.spin_once(self.node, timeout_sec=min(0.01, max(0.0, nxt - time.monotonic())))
        if self.rows is not None:
            self.rows.append({"t": round(k / self.rate, 3), "lin_raw": self.sent[0], "ang_raw": self.sent[1], **self._state(),
                              "rl_onnx": getattr(self, "tag", "")})   # 보정: 모델 파일 이름 · bypass · direct

    def _state(self):
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
        self.rclpy.shutdown()
