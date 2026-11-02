"""02 — 센서 싱크 (real, PC에서 실행).

로봇에 sim과 같은 정사각형 프로파일을 보내고 `/odom`·`/imu`를 CSV로 기록한다.
2초 정지 → 왼쪽 사각형(반시계) → 180° 턴 → 같은 사각형 역주행
→ 2초 정지. 각 변: 0.5 m 전진 → 0.25 s 정지 → 제자리 90도 회전 → 0.25 s 정지.
sim 쪽 `DefaultSensorSim.py`와 같은 `excitation/CalibExcitation.py`를 쓴다.

사전:
  Pi: base + cmd_manager, 제어 MANUAL
      cd ~/ros2/tool && ./run-cli base start && ./run-cli mode manual
  PC: source /opt/ros/lyrical/setup.bash
      export ROS_DOMAIN_ID=25
      source tool/peers.sh

실행:
  ./run-calibration.sh sensor-sync-real --label real_floor
출력 기본: artifacts/calibration/<시각>-<label>/raw.csv (--label 없으면 <시각>-02-real)
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ISAAC))

from calibration.Calibrate import Calibrate  # noqa: E402
from calibration.sensor.Sensor import Sensor  # noqa: E402


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DefaultSensorReal(Node, Sensor):
    """real 실행기 — Twist 발행 + 토픽 스냅샷으로 행을 만든다."""

    TAG = "02-real"

    def __init__(self, label: str | None = None):
        Node.__init__(self, "sensor_sync")
        Sensor.__init__(self)
        self.label = label or "real"
        self.run_name = label or self.TAG
        self.state = None
        self.odom = None
        self.imu_w = 0.0
        self.imu_seen = False
        self.imu_stamp = None
        self.wheel_l_vel = self.wheel_r_vel = 0.0
        self.wheel_l_pos = self.wheel_r_pos = 0.0
        self.joint_seen = False
        self.joint_stamp = None
        self.out = (0.0, 0.0)
        self.pub = self.create_publisher(Twist, "/cmd_manual_vel", 10)
        # /state_control은 TRANSIENT_LOCAL (latched) — 기본 volatile이면 현재 상태를 못 받음
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/state_control", lambda m: setattr(self, "state", m.data), state_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Imu, "/imu", self.on_imu, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint, 10)
        self.create_subscription(Twist, "/cmd_vel_base", self.on_out, 10)

    def on_odom(self, m):
        self.odom = m

    def on_imu(self, m):
        self.imu_w = m.angular_velocity.z
        self.imu_seen = True
        self.imu_stamp = m.header.stamp

    def on_joint(self, m):
        self.joint_seen = True
        self.joint_stamp = m.header.stamp
        for name, pos, vel in zip(m.name, m.position, m.velocity):
            if name == "wheel_left_joint":
                self.wheel_l_pos, self.wheel_l_vel = pos, vel
            elif name == "wheel_right_joint":
                self.wheel_r_pos, self.wheel_r_vel = pos, vel

    def on_out(self, m):
        self.out = (m.linear.x, m.angular.z)

    def send(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    @staticmethod
    def _sec(ts):
        if ts is None:
            return float("nan")
        if hasattr(ts, "nanoseconds"):  # rclpy Time
            return ts.nanoseconds / 1e9
        return float(ts.sec) + float(ts.nanosec) * 1e-9  # builtin_interfaces Time

    def collect(self, t: float, v: float, w: float) -> list:
        o = self.odom
        p = o.pose.pose
        return [round(t, 3), v, w, self.out[0], self.out[1],
                p.position.x, p.position.y, yaw_of(p.orientation),
                o.twist.twist.linear.x, o.twist.twist.angular.z,
                self.imu_w,
                self.wheel_l_vel, self.wheel_r_vel,
                self.wheel_l_pos, self.wheel_r_pos,
                self.label,
                self._sec(self.get_clock().now()),
                self._sec(o.header.stamp),
                self._sec(self.imu_stamp),
                self._sec(self.joint_stamp)]

    @classmethod
    def main(cls, argv=None):
        parser = argparse.ArgumentParser(description="02: real 센서 싱크")
        parser.add_argument("--output", default=None,
                            help="정확한 경로 (기본: artifacts/calibration/<시각>-<label>/raw.csv)")
        parser.add_argument("--label", default=None,
                            help="CSV label · 기본 폴더 이름 (<시각>-<label>) — 없으면 real / 02-real")
        parser.add_argument("--speed", type=float, default=0.15,
                            help="기준 전진 속도 [m/s] (변마다 1.0/0.67/1.33/0.8 배)")
        parser.add_argument("--turn", type=float, default=1.0)
        parser.add_argument("--side", type=float, default=0.5, help="정사각형 한 변 [m]")
        parser.add_argument("--rate", type=float, default=20.0)
        parser.add_argument("--check-only", action="store_true", help="상태만 확인하고 종료 (로봇 안 움직임)")
        parser.add_argument("--excitation", default=None,
                            help="측정 경로 클래스 (기본 DefaultCalibExcitation: 사각형→180°→역주행): pkg.mod.Class 또는 path/to/file.py[:Class] — calibration.excitation.CalibExcitation 서브클래스")
        args = parser.parse_args(argv)
        excitation = Calibrate.load_excitation(args.excitation)   # 로봇 움직이기 전에 실패하게

        rclpy.init()
        node = cls(label=args.label)
        try:
            end = time.monotonic() + 5.0
            while node.state is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(f"[02-real] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')} · state={node.state}")
            if args.check_only:
                return 0
            if node.state != "MANUAL":
                print("[02-real] 거부: MANUAL 아님 — Pi에서 ./run-cli mode manual 먼저")
                return 2
            end = time.monotonic() + 3.0
            while node.odom is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.odom is None:
                print("[02-real] 거부: /odom 없음 — base/드라이버 확인")
                return 2

            rate = args.rate
            t0 = time.monotonic()
            out = args.output   # None → Calibrate가 artifacts/calibration/<시각>-02-real/raw.csv
            segs = excitation.make_profile(args.speed, args.turn, args.side)
            total = excitation.total(segs)
            speeds_txt = node.move_speeds_text(segs)
            print(f"[02-real] 시작 · {total:.0f}s · {rate:.0f}Hz · side={args.side:.2f}m "
                  f"· speeds {speeds_txt} m/s · left square → 180° turn → reverse "
                  f"· {node.label} → {out or f'artifacts/calibration/<시각>-{node.run_name}/'}")

            def pace(k: int, t: float, _t0=t0, _rate=rate) -> None:
                next_t = _t0 + (k + 1) / _rate
                while time.monotonic() < next_t:
                    rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, next_t - time.monotonic())))

            cal = Calibrate(node)
            cal.run(segs, total, out, 1.0 / rate, pace=pace)

            for _ in range(3):
                node.send(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
            if not node.imu_seen:
                print("[02-real] 경고: /imu 없음 — w_gyro가 0")
            if not node.joint_seen:
                print("[02-real] 경고: /joint_states 없음 — 바퀴 raw가 0")
            return 0
        except KeyboardInterrupt:
            for _ in range(3):
                node.send(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
            print("\n[02-real] 중단 — 정지")
            return 130
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(DefaultSensorReal.main())
