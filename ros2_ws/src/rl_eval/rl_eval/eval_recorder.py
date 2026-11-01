"""Before/After 기록기 — 받을 수 있는 건 다 받아 입력 명령 1개당 CSV 1줄로 쓴다.

한 줄 = 입력(/cmd_vel_rl_in) 1개 + 그 입력에 corrector가 낸 응답(/cmd_vel_rl_out · /rl_debug)
+ 입력이 온 순간의 센서·상태 스냅샷. 응답은 다음 입력이 오기 전까지 기다렸다가 짝지어 쓴다
(그래서 한 스텝(≈50 ms) 늦게 써진다). 응답이 없으면(corrector 없음) out 칸은 nan.
아직 한 번도 못 받은 토픽 칸은 nan / 빈 문자열 (0으로 꾸미지 않는다).

칸 — 앞 20개는 isaac calibration CSV와 이름·순서가 같다:
  t, lin_raw, ang_raw, lin_out, ang_out, odom_x, odom_y, odom_yaw, v_enc, w_enc, w_gyro,
  wheel_l_vel, wheel_r_vel, wheel_l_pos, wheel_r_pos, label, t_ros, t_odom, t_imu, t_joint,
  model_dv, model_dw, t_out, cmd_base_lin, cmd_base_ang, wheel_l_cur, wheel_r_cur,
  motor_l_temp, motor_r_temp, imu_wx, imu_wy, imu_ax, imu_ay, imu_az, imu_yaw,
  battery_v, battery_pct, state_control, state_rl, state_estop, diag_level, diag_msg,
  rl_enable, rl_model, rl_policy

  t = t_ros = 입력 수신 시각(epoch s) · t_odom/t_imu/t_joint = 각 메시지 stamp · t_out = 응답 수신 시각
  model_dv/dw = 정책 원출력(/rl_debug, clip 전) · lin/ang_out − lin/ang_raw = 실제 적용된 보정
  cmd_base = cmd_manager가 드라이버로 보낸 명령 (/cmd_vel_base) · cur = 모터 전류 [A] (/joint_states effort)
  motor temp · diag = /diagnostics(opencr) · battery = /battery_state · state_* = /state_* (latched)
  rl_enable · rl_model = 그 순간 /rl_corrector 파라미터 (시작값은 launch, 바뀌면 /parameter_events로 따라감)
  rl_policy = 실제 구간: 모델 파일 이름(enable이고 파일이 있을 때) 또는 bypass — report.py가 이걸로 A/B를 나눈다

비교 그림: ros2_ws/report.py before.csv after.csv  (→ report.png + 지표 표)
"""
import csv
import math
import os

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_parameter_events
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Bool, Float32MultiArray, String

NAN = float('nan')
HEADER = ['t', 'lin_raw', 'ang_raw', 'lin_out', 'ang_out', 'odom_x', 'odom_y', 'odom_yaw',
          'v_enc', 'w_enc', 'w_gyro', 'wheel_l_vel', 'wheel_r_vel', 'wheel_l_pos', 'wheel_r_pos',
          'label', 't_ros', 't_odom', 't_imu', 't_joint',
          'model_dv', 'model_dw', 't_out', 'cmd_base_lin', 'cmd_base_ang', 'wheel_l_cur', 'wheel_r_cur',
          'motor_l_temp', 'motor_r_temp', 'imu_wx', 'imu_wy', 'imu_ax', 'imu_ay', 'imu_az', 'imu_yaw',
          'battery_v', 'battery_pct', 'state_control', 'state_rl', 'state_estop', 'diag_level', 'diag_msg',
          'rl_enable', 'rl_model', 'rl_policy']
TEXT = ('label', 'state_control', 'diag_msg', 'rl_model', 'rl_policy')
CORRECTOR = '/rl_corrector'   # 파라미터를 따라갈 노드 (rl-only.launch.py의 corrector 이름)
# state_manager와 같은 QoS — latched라 늦게 붙어도 현재 값을 받는다
LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class EvalRecorder(Node):
    def __init__(self):
        super().__init__('eval_recorder')
        self.declare_parameter('output', '/tmp/rl_eval.csv')
        self.declare_parameter('label', 'rl')
        self.declare_parameter('rl_enable', False)   # corrector와 같은 launch 값 (시작 상태)
        self.declare_parameter('rl_model', '')
        self.label = self.get_parameter('label').value
        # 최신값 (토픽마다 받을 때 갱신) — 처음엔 nan
        self.s = {k: NAN for k in HEADER if k not in TEXT}
        self.s.update(state_control='', diag_msg='')
        self.set_policy(enable=bool(self.get_parameter('rl_enable').value), model=self.get_parameter('rl_model').value)
        self.pending = None   # 응답을 기다리는 입력 행
        # 줄 단위 버퍼 — 한 줄 쓸 때마다 디스크로 (테스트 중 tail -f로 보이고, 강제 종료돼도 쓴 줄은 남음)
        self.f = open(self.get_parameter('output').value, 'w', newline='', buffering=1)
        self.w = csv.writer(self.f)
        self.w.writerow(HEADER)

        sub = self.create_subscription
        sub(Twist, 'cmd_vel_raw', self.cb_raw, 10)
        sub(Twist, 'cmd_vel', self.cb_out, 10)
        sub(Float32MultiArray, 'rl_debug', self.cb_debug, 10)
        sub(Twist, 'cmd_vel_base', self.cb_base, 10)
        sub(Odometry, 'odom', self.cb_odom, 10)
        sub(Imu, 'imu', self.cb_imu, 10)
        sub(JointState, 'joint_states', self.cb_joint, 10)
        sub(BatteryState, 'battery_state', self.cb_battery, 10)
        sub(DiagnosticArray, 'diagnostics', self.cb_diag, 10)
        sub(String, 'state_control', lambda m: self.s.update(state_control=m.data), LATCHED)
        sub(Bool, 'state_rl', lambda m: self.s.update(state_rl=int(m.data)), LATCHED)
        sub(Bool, 'state_estop', lambda m: self.s.update(state_estop=int(m.data)), LATCHED)
        sub(ParameterEvent, '/parameter_events', self.cb_params, qos_profile_parameter_events)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------ 행
    def cb_raw(self, msg):
        self.finish()   # 직전 입력 행 — 그 뒤에 온 응답과 짝지어 쓴다
        t = self.now_s()
        row = dict(self.s, t=t, t_ros=t, lin_raw=msg.linear.x, ang_raw=msg.angular.z, label=self.label)
        row.update(lin_out=NAN, ang_out=NAN, model_dv=NAN, model_dw=NAN, t_out=NAN)
        self.pending = row

    def cb_out(self, msg):
        if self.pending is not None and math.isnan(self.pending['t_out']):   # 입력 뒤 첫 응답만
            self.pending.update(lin_out=msg.linear.x, ang_out=msg.angular.z, t_out=self.now_s())

    def cb_debug(self, msg):
        # corrector track: [0, cmd_v, cmd_w, v_enc, w_enc, w_gyro, 0, 0, cmd_v, cmd_w, model_dv, model_dw, ...]
        if self.pending is not None and len(msg.data) >= 12 and math.isnan(self.pending['model_dv']):
            self.pending.update(model_dv=float(msg.data[10]), model_dw=float(msg.data[11]))

    def finish(self):
        if self.pending is None:
            return
        self.w.writerow([self.pending[k] for k in HEADER])
        self.pending = None

    # ------------------------------------------------------------ corrector 구간 (A/B)
    def set_policy(self, enable=None, model=None):
        if enable is not None:
            self.s['rl_enable'] = int(enable)
        if model is not None:
            self.s['rl_model'] = model
        m = self.s['rl_model']
        self.s['rl_policy'] = os.path.basename(m) if self.s['rl_enable'] and m and os.path.isfile(m) else 'bypass'

    def cb_params(self, msg):
        if msg.node != CORRECTOR:
            return
        for p in list(msg.new_parameters) + list(msg.changed_parameters):
            if p.name == 'enable':
                self.set_policy(enable=p.value.bool_value)
            elif p.name == 'model_path':
                self.set_policy(model=p.value.string_value)

    # ------------------------------------------------------------ 센서 최신값
    def cb_base(self, msg):
        self.s.update(cmd_base_lin=msg.linear.x, cmd_base_ang=msg.angular.z)

    def cb_odom(self, msg):
        p = msg.pose.pose
        self.s.update(odom_x=p.position.x, odom_y=p.position.y, odom_yaw=yaw_of(p.orientation),
                      v_enc=msg.twist.twist.linear.x, w_enc=msg.twist.twist.angular.z,
                      t_odom=stamp_s(msg.header))

    def cb_imu(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.s.update(w_gyro=g.z, imu_wx=g.x, imu_wy=g.y, imu_ax=a.x, imu_ay=a.y, imu_az=a.z,
                      imu_yaw=yaw_of(msg.orientation), t_imu=stamp_s(msg.header))

    def cb_joint(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        for side, name in (('l', 'wheel_left_joint'), ('r', 'wheel_right_joint')):
            i = idx.get(name)
            if i is None:
                continue
            if i < len(msg.velocity):
                self.s[f'wheel_{side}_vel'] = msg.velocity[i]
            if i < len(msg.position):
                self.s[f'wheel_{side}_pos'] = msg.position[i]
            if i < len(msg.effort):
                self.s[f'wheel_{side}_cur'] = msg.effort[i]
        self.s['t_joint'] = stamp_s(msg.header)

    def cb_battery(self, msg):
        self.s.update(battery_v=msg.voltage, battery_pct=msg.percentage)

    def cb_diag(self, msg):
        for st in msg.status:
            if st.name != 'opencr':
                continue
            kv = {v.key: v.value for v in st.values}
            for col, key in (('motor_l_temp', 'left_temp_C'), ('motor_r_temp', 'right_temp_C')):
                try:
                    self.s[col] = float(kv[key])
                except (KeyError, ValueError):
                    pass
            self.s.update(diag_level=int.from_bytes(st.level, 'little') if isinstance(st.level, bytes)
                          else int(st.level), diag_msg=st.message)


def main(args=None):
    rclpy.init(args=args)
    node = EvalRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()   # 응답을 기다리던 마지막 입력 행
        node.f.close()
        node.destroy_node()
        rclpy.try_shutdown()
