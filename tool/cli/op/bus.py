"""Bus — 발행·액션·프로세스 실행. 한 번 실행 명령(cmd_*)과 메뉴가 같이 쓴다."""
from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped

from .procs import MAP_DIR, SERVICES, Procs
from .terminal import DIM, GREEN, RED, YELLOW, Terminal

# 수동 주행 명령은 cmd_manager(mux)의 수동 입력으로 — /state_mode=MANUAL일 때만 바퀴로 감 (ARCH.md)
CMD_VEL = os.environ.get('TOOL_CMD_VEL', '/cmd_manual_vel')
# Nav2(Lyrical)는 TwistStamped를 낸다. mock_base·자체 드라이버가 Twist면 0.
CMD_VEL_STAMPED = os.environ.get('TOOL_CMD_VEL_STAMPED', '0') == '1'


def yaw_quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Bus:
    def __init__(self, node, state, ui, term: Terminal):
        self.node = node
        self.state = state
        self.ui = ui
        self.term = term
        self.procs = Procs()
        msg_type = TwistStamped if CMD_VEL_STAMPED else Twist
        self._vel_pub = node.create_publisher(msg_type, CMD_VEL, 10)
        self._init_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        from std_msgs.msg import Empty
        self._reboot_pub = node.create_publisher(Empty, '/opencr/reboot', 10)
        from std_msgs.msg import Bool, String
        self._req = {'mode': (node.create_publisher(String, '/cmd_manual_mode', 10), String),
                     'rl': (node.create_publisher(Bool, '/cmd_manual_rl', 10), Bool),
                     'estop': (node.create_publisher(Bool, '/cmd_manual_estop', 10), Bool)}
        self._nav_client = None
        self._goal_handle = None

    def _poll(self, seconds: float = 0.2):
        self.node.poll(seconds)

    # ------------------------------------------------------------- 상태
    def service_rows(self) -> list[tuple[str, str, bool, str]]:
        """(이름, 라벨, 실행 중, 부가정보)."""
        rows = []
        for name, svc in SERVICES.items():
            pid = self.procs.pid(name)
            extra = ''
            if pid:
                up = self.procs.uptime(name) or 0
                extra = f'pid {pid} · {int(up // 60)}분 {int(up % 60)}초'
                info = self.procs.info(name)
                if info:
                    extra += f' · {os.path.basename(os.path.dirname(info))}'
            rows.append((name, svc.label, pid is not None, extra))
        return rows

    def status_lines(self) -> list[str]:
        lines = ['[프로세스]']
        for _, label, on, extra in self.service_rows():
            mark = self.term.paint('●', GREEN) if on else self.term.paint('○', DIM)
            lines.append(f' {mark} {label + ' ' * max(0, 16 - self.term.disp_len(label))} {extra}')
        lines += ['', '[로봇]', f' {self.state.robot_txt()}', f' {self.state.domain_txt()}']
        lines += ['', '[토픽]',
                  f' odom  {self.state.odom_txt()}',
                  f' scan  {self.state.scan_txt()}',
                  f' map   {self.state.map_txt()}',
                  f' amcl  {self.state.amcl_txt()}']
        if self.state.goal_status:
            lines.append(f' 목표  {self.state.goal_status}')
        return lines

    def cmd_status(self, args) -> int:
        self._poll(2.5)
        print('\n'.join(self.status_lines()))
        return 0

    # ------------------------------------------------------------- 프로세스
    def start(self, name: str, map_yaml: str | None = None) -> int:
        ok, msg = self.procs.start(name, map_yaml)
        print(msg)
        return 0 if ok else 1

    def stop(self, name: str) -> int:
        ok, msg = self.procs.stop(name)
        print(msg)
        return 0 if ok else 1

    def cmd_svc(self, args) -> int:
        name = args.command
        if args.action == 'start':
            map_yaml = None
            if name == 'nav':
                path = self.procs.resolve_map(args.map)
                if path is None:
                    print(f'지도 없음: {args.map or "(최근 지도)"} — {MAP_DIR} 확인 · `map list`', file=sys.stderr)
                    return 1
                map_yaml = str(path)
            return self.start(name, map_yaml)
        if args.action == 'stop':
            return self.stop(name)
        if args.action == 'restart':
            info = self.procs.info(name) or None
            self.stop(name)
            return self.start(name, info)
        if args.action == 'log':
            print(self.procs.tail(name, args.lines))
            return 0
        on = self.procs.running(name)
        print(f'{SERVICES[name].label}: {"실행 중" if on else "꺼짐"}')
        return 0 if on else 3

    def cmd_stop_all(self, args) -> int:
        self.send_velocity(0.0, 0.0)
        for name in reversed(list(SERVICES)):
            if self.procs.running(name):
                self.stop(name)
        print('전부 종료')
        return 0

    # ------------------------------------------------------------- 지도
    @staticmethod
    def default_map_name() -> str:
        return time.strftime('%Y%m%d_%H%M%S')

    def cmd_map(self, args) -> int:
        if args.action == 'list':
            maps = self.procs.maps()
            if not maps:
                print(f'지도 없음 ({MAP_DIR})')
            for name, path in maps:
                stamp = time.strftime('%m-%d %H:%M', time.localtime(path.stat().st_mtime))
                print(f'{name:<24} {stamp}  {path}')
            return 0
        name = args.name or self.default_map_name()
        from .procs import IS_ROBOT
        if not IS_ROBOT:
            print(self.term.paint(f'주의: 여기는 로봇이 아님 — 지도는 이 컴퓨터의 {MAP_DIR}에 저장됩니다 '
                                  '(Nav는 로봇에서 도니 로봇에서 저장하는 게 맞음)', YELLOW))
        if self.state.map.age() is None:
            print(self.term.paint('/map 수신 없음 — SLAM이 지도를 내고 있지 않습니다 (라이다 /scan이 있어야 지도가 생김)', RED))
            return 1
        if not self.procs.running('slam') and IS_ROBOT:
            print(self.term.paint('주의: SLAM이 꺼져 있음 — /map을 내는 노드(map_server 등)가 있어야 저장됩니다', YELLOW))
        ok, msg = self.procs.save_map(name)
        print(msg)
        return 0 if ok else 1

    # ------------------------------------------------------------- 상태 요청 (state_manager)
    def request(self, key: str, value) -> bool:
        """/cmd_manual_<key> 발행 → /state_<key>가 그 값이 될 때까지 (최대 3초) 기다림."""
        pub, typ = self._req[key]
        end = time.monotonic() + 5.0  # 방금 만든 발행자 — state_manager와 연결될 때까지
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            self._poll(0.2)
        if pub.get_subscription_count() == 0:
            print(self.term.paint('state_manager 없음 — 베이스(tb-only)가 떠 있는지 확인', RED))
            return False
        pub.publish(typ(data=value))
        end = time.monotonic() + 3.0
        while getattr(self.state, f'st_{key}') != value and time.monotonic() < end:
            self._poll(0.1)
        return getattr(self.state, f'st_{key}') == value

    def ensure_mode(self, mode: str) -> None:
        """주행 명령 전에 모드 맞추기 (텔레오퍼·drive = MANUAL, 목표 = AUTO). 이미 그 모드면 아무것도 안 함."""
        end = time.monotonic() + 2.0  # 새로 켠 CLI는 latched /state_mode를 받기 전 — 잠깐 기다림
        while self.state.st_mode is None and time.monotonic() < end:
            self._poll(0.1)
        if self.state.st_mode != mode and self.request('mode', mode):
            print(f'모드 → {mode}')

    def cmd_mode(self, args) -> int:
        ok = self.request('mode', args.value.upper())
        print(f'모드: {self.state.st_mode}' if ok else self.term.paint('모드 변경 실패', RED))
        return 0 if ok else 1

    def cmd_rl(self, args) -> int:
        ok = self.request('rl', args.value == 'on')
        if ok and self.state.st_rl:
            print('RL 켬 — PC에서 ros2 launch tb3_bringup rl-only.launch.py 가 떠 있어야 바퀴가 움직입니다')
        print(f'RL: {"on" if self.state.st_rl else "off"}' if ok else self.term.paint('RL 변경 실패', RED))
        return 0 if ok else 1

    def cmd_estop(self, args) -> int:
        on = args.value == 'on'
        if on:
            self.send_velocity(0.0, 0.0)
        ok = self.request('estop', on)
        print(f'비상정지: {"ON" if self.state.st_estop else "OFF"}' if ok else self.term.paint('비상정지 변경 실패', RED))
        return 0 if ok else 1

    # ------------------------------------------------------------- 주행
    def send_velocity(self, linear: float, angular: float) -> None:
        if CMD_VEL_STAMPED:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            t = msg.twist
        else:
            msg = t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self._vel_pub.publish(msg)
        self.state.last_cmd = (linear, angular)

    def cmd_drive(self, args) -> int:
        self.ensure_mode('MANUAL')
        v = {'forward': args.speed, 'back': -args.speed}.get(args.direction, 0.0)
        w = {'left': args.angular, 'right': -args.angular}.get(args.direction, 0.0)
        end = time.monotonic() + (args.duration if args.direction != 'stop' else 0.0)
        try:
            while time.monotonic() < end and rclpy.ok():
                self.send_velocity(v, w)
                self._poll(0.1)
        finally:
            for _ in range(3):
                self.send_velocity(0.0, 0.0)
                self._poll(0.05)
        print(f'{args.direction} 완료 → 정지 ({CMD_VEL})')
        return 0

    def cmd_teleop(self, args) -> int:
        if not sys.stdin.isatty():
            print('거부: teleop은 터미널에서 실행해야 합니다', file=sys.stderr)
            return 2
        self.ensure_mode('MANUAL')
        from .dialogs import KeyReader
        linear = angular = 0.0
        lin_speed, ang_speed = args.linear, args.angular
        last_send = 0.0
        last_key, key_count = '-', 0
        con = getattr(self.term, 'out', None) or sys.stdout  # 캡처를 우회해 화면에 직접
        alias = {'up': 'w', 'down': 'x', 'left': 'a', 'right': 'd', ' ': 's'}
        try:
            with KeyReader(self.term) as keys:
                while rclpy.ok():
                    cols, rows = self.term.size()
                    w = min(cols - 4, 64)
                    left = max(1, (cols - w) // 2 + 1)
                    top = max(1, (rows - 11) // 2 + 1)
                    key_txt = f'키 {last_key} ×{key_count}' if key_count else '키 입력 대기 중'
                    rows_txt = [f'텔레오퍼 → {CMD_VEL}',
                                f'명령 v={linear:+.2f} m/s ω={angular:+.2f} rad/s',
                                f'odom {self.state.odom_txt()}',
                                f'scan {self.state.scan_txt()}',
                                f'map  {self.state.map_txt()}',
                                f'{key_txt} · 설정 v {lin_speed:.2f} m/s · ω {ang_speed:.2f} rad/s',
                                self.term.paint('w/x/a/d·방향키 이동 · s/space 정지 · e/c 속도 · q/Esc 종료', DIM)]
                    con.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')
                    for i, line in enumerate(rows_txt):
                        con.write(f'\033[{top + 1 + i};{left}H│' + self.term.fit(line, w - 2) + '│')
                    con.write(f'\033[{top + 1 + len(rows_txt)};{left}H└' + '─' * (w - 2) + '┘')
                    con.flush()
                    now = time.monotonic()
                    if now - last_send >= 0.1:
                        self.send_velocity(linear, angular)
                        last_send = now
                    key = keys.read(0.1)
                    self._poll(0.01)
                    if key is None:
                        continue
                    if key in ('q', 'Q', 'esc', 'eof', '\x03'):
                        break
                    key = alias.get(key, key)
                    key_count += 1
                    last_key = 'space' if key == 's' else key
                    if key == 'w':
                        linear, angular = lin_speed, 0.0
                    elif key == 'x':
                        linear, angular = -lin_speed, 0.0
                    elif key == 'a':
                        linear, angular = 0.0, ang_speed
                    elif key == 'd':
                        linear, angular = 0.0, -ang_speed
                    elif key == 's':
                        linear = angular = 0.0
                    elif key == 'e':
                        lin_speed = min(lin_speed + 0.05, 0.30)
                    elif key == 'c':
                        lin_speed = max(lin_speed - 0.05, 0.05)
        finally:
            for _ in range(3):
                self.send_velocity(0.0, 0.0)
                self._poll(0.05)
            print('텔레오퍼 종료 — 바퀴 정지')
        return 0

    # ------------------------------------------------------------- Nav2
    def set_initial_pose(self, x: float, y: float, yaw_deg: float) -> int:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qz, qw = yaw_quat(math.radians(yaw_deg))
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        end = time.monotonic() + 10.0  # 방금 만든 발행자는 amcl과 연결(discovery)되기 전 — 기다린다
        while self._init_pub.get_subscription_count() == 0 and time.monotonic() < end:
            self._poll(0.2)
        if self._init_pub.get_subscription_count() == 0:
            print(self.term.paint('/initialpose 구독자 없음 — Nav(amcl)가 떠 있는지 확인', RED))
            return 1
        before = self.state.amcl_rate.age()
        for _ in range(3):
            self._init_pub.publish(msg)
            self._poll(0.3)
        end = time.monotonic() + 3.0
        while time.monotonic() < end and self.state.amcl_rate.age() in (None, before):
            self._poll(0.1)
        got = self.state.amcl_rate.age() not in (None, before)
        print(f'초기 위치 발행: x={x:.2f} y={y:.2f} θ={yaw_deg:.0f}° (/initialpose) → '
              + ('amcl 반영: ' + self.state.amcl_txt() if got else
                 self.term.paint('amcl 응답 없음 — 아직 기동 중이면 잠시 후 다시', YELLOW)))
        return 0 if got else 1

    def cmd_initpose(self, args) -> int:
        return self.set_initial_pose(args.x, args.y, args.yaw)

    def _client(self):
        if self._nav_client is None:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self._nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        return self._nav_client

    def send_goal(self, x: float, y: float, yaw_deg: float, wait: bool = True) -> int:
        from nav2_msgs.action import NavigateToPose
        client = self._client()
        end = time.monotonic() + 5.0
        while not client.server_is_ready() and time.monotonic() < end:
            self._poll(0.2)
        if not client.server_is_ready():
            print(self.term.paint('navigate_to_pose 서버 없음 — Nav 또는 SLAM 중 Nav를 먼저 켜세요', RED))
            return 1
        self.ensure_mode('AUTO')  # Nav2 /cmd_vel은 AUTO일 때만 바퀴로 감
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz, qw = yaw_quat(math.radians(yaw_deg))
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = qz, qw

        def on_feedback(fb):
            d = fb.feedback.distance_remaining
            self.state.goal_status = f'({x:.2f},{y:.2f}) 남은 {d:.2f}m'

        self._poll(0.5)  # 서버가 보여도 goal 서비스 연결이 덜 됐을 수 있다 — 첫 요청이 사라지지 않게
        fut = client.send_goal_async(goal, feedback_callback=on_feedback)
        end = time.monotonic() + 10.0
        while not fut.done() and time.monotonic() < end:
            self._poll(0.1)
        if not fut.done():
            print(self.term.paint('목표 응답 없음 (10초) — Nav 로그 확인: ./run-cli nav log', RED))
            return 1
        handle = fut.result()
        if not handle.accepted:
            print(self.term.paint('목표 거부됨 — bt_navigator가 거절 (Nav 기동 중? 초기 위치?)', RED))
            return 1
        self._goal_handle = handle
        self.state.goal_status = f'({x:.2f},{y:.2f}) 이동 중'
        print(f'목표 전송: x={x:.2f} y={y:.2f} θ={yaw_deg:.0f}°')
        if not wait:
            return 0
        res_fut = handle.get_result_async()
        try:
            while not res_fut.done() and rclpy.ok():
                self._poll(0.2)
        except KeyboardInterrupt:
            self.cancel_goal()
            return 130
        code = res_fut.result().status  # 4=SUCCEEDED 5=CANCELED 6=ABORTED
        txt = {4: '도착', 5: '취소됨', 6: '실패(ABORTED)'}.get(code, f'status {code}')
        self.state.goal_status = f'({x:.2f},{y:.2f}) {txt}'
        print(f'결과: {txt}')
        return 0 if code == 4 else 1

    def cancel_goal(self) -> int:
        if self._goal_handle is None:
            print('취소할 목표 없음 (이 CLI에서 보낸 목표만 취소 가능 — RViz 목표는 RViz에서)')
            return 0
        fut = self._goal_handle.cancel_goal_async()
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            self._poll(0.1)
        self._goal_handle = None
        self.state.goal_status = '취소됨'
        self.send_velocity(0.0, 0.0)
        print('목표 취소')
        return 0

    def cmd_goal(self, args) -> int:
        if args.cancel:
            return self.cancel_goal()
        if args.x is None or args.y is None:
            print('goal x y [yaw°] 또는 goal --cancel', file=sys.stderr)
            return 2
        return self.send_goal(args.x, args.y, args.yaw, wait=not args.no_wait)

    # ------------------------------------------------------------- 모터
    def cmd_reboot_motors(self, args) -> int:
        """모터 에러(과열·과부하 등) 해제 — /opencr/reboot → opencr_driver가 펌웨어에 R (양쪽 재부팅)."""
        from std_msgs.msg import Empty
        end = time.monotonic() + 10.0  # 방금 만든 발행자 — 드라이버와 연결될 때까지
        while self._reboot_pub.get_subscription_count() == 0 and time.monotonic() < end:
            self._poll(0.2)
        if self._reboot_pub.get_subscription_count() == 0:
            print(self.term.paint('opencr_driver 없음 — 베이스(tb-only)가 떠 있는지 확인', RED))
            return 1
        self.send_velocity(0.0, 0.0)
        self._reboot_pub.publish(Empty())
        print('모터 재부팅 요청 (R) — 바퀴가 잠깐 힘이 빠졌다가 돌아옵니다')
        self._poll(3.0)  # 재부팅(바퀴당 ~1초) 후 진단 갱신
        print(f'로봇: {self.state.robot_txt()}')
        return 0 if self.state.robot_parts()[0] == 0 else 1

    # ------------------------------------------------------------- 노드
    def node_names(self) -> list[str]:
        self._poll(0.5)
        names = self.node.get_node_names_and_namespaces()
        return sorted(f'{ns.rstrip("/")}/{n}' for n, ns in names if n != self.node.get_name())

    def cmd_nodes(self, args) -> int:
        names = self.node_names()
        print('\n'.join(names) if names else '노드 없음 (같은 ROS_DOMAIN_ID인지 확인)')
        return 0
