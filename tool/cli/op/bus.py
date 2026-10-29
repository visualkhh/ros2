"""Bus — 발행·액션·프로세스 실행. 한 번 실행 명령(cmd_*)과 메뉴가 같이 쓴다."""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from std_msgs.msg import Empty

from .procs import SERVICES
from .terminal import DIM, GREEN, RED, YELLOW, Terminal

# 수동 주행 명령은 cmd_manager(mux)의 수동 입력으로 — /state_control=MANUAL일 때만 바퀴로 감 (ARCH.md)
CMD_VEL = os.environ.get('TOOL_CMD_VEL', '/cmd_manual_vel')
# Nav2(Lyrical)는 TwistStamped를 낸다. cmd_manager·드라이버가 Twist라 0.
CMD_VEL_STAMPED = os.environ.get('TOOL_CMD_VEL_STAMPED', '0') == '1'
# 수동 주행 속도 상한 (텔레오퍼 e · 메뉴 속도 설정) — Waffle Pi 사양 = 펌웨어 바퀴 상한 8 rad/s × 0.033 m
MAX_LIN, MAX_ANG = 0.26, 1.82
# 텔레오퍼 키 → (전후, 회전) 방향. 사선(둘 다 0 아님) = 설정 선속도 그대로 + 반경 DIAG_RADIUS 원호
#   (ω = v / R → 속도를 바꿔도 휘는 모양이 같음). 0.20 m/s까지 속도 그대로, 그 위는 바깥 바퀴가 상한이라
#   opencr_driver가 두 바퀴를 같은 비율로 줄임 (모양 유지).
# 후진 사선은 회전 부호 반대 (teleop_twist_keyboard와 같음: z = 뒤로 가며 왼쪽으로 휘어짐)
TELEOP_MOVES = {'w': (1, 0), 'x': (-1, 0), 'a': (0, 1), 'd': (0, -1), 's': (0, 0),
                'q': (1, 1), 'e': (1, -1), 'z': (-1, -1), 'c': (-1, 1)}
DIAG_RADIUS = 0.5   # [m] 사선 원호 반경
DRIVER_MISSING = 'opencr_driver 없음 — 베이스(tb-only)가 떠 있는지 확인'


def yaw_quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Bus:
    def __init__(self, node, state, ui, term: Terminal):
        self.node = node
        self.state = state
        self.ui = ui
        self.term = term
        self.procs = node.procs
        msg_type = TwistStamped if CMD_VEL_STAMPED else Twist
        self._vel_pub = node.create_publisher(msg_type, CMD_VEL, 10)
        self._init_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._reboot_pub = node.create_publisher(Empty, '/opencr/reboot', 10)
        self._beep_pub = node.create_publisher(Empty, '/cmd_beep', 10)
        from std_msgs.msg import Int32MultiArray
        self._beep_tone_pub = node.create_publisher(Int32MultiArray, '/cmd_beep_tone', 10)
        self._say_pub = None   # /cmd_say — say_text 첫 사용 때 생성
        from std_msgs.msg import Bool, String
        self._req = {'control': (node.create_publisher(String, '/cmd_manual_control', 10), String),
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
            on = self.procs.running(name)
            extra = '' if pid or not on else f'{self.procs.host} (원격)'
            if pid:
                up = self.procs.uptime(name) or 0
                extra = f'pid {pid} · {int(up // 60)}분 {int(up % 60)}초'
                info = self.procs.info(name)
                if info:
                    extra += f' · {os.path.basename(os.path.dirname(info))}'
            rows.append((name, svc.label, on, extra))
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
    def base_ready(self) -> bool:
        """SLAM·Nav는 베이스(odom·scan·TF·로봇 모델)가 있어야 의미가 있음 — 없으면 안내하고 False."""
        if self.procs.running('base'):
            return True
        print(self.term.paint('베이스가 꺼져 있음 — 먼저 ./run-cli base start (메뉴 2) 베이스 → 1) 시작)', RED))
        print('  (베이스 없이 SLAM·Nav를 켜면 odom·scan이 없어 지도·RViz에 아무것도 안 나옵니다)')
        return False

    def start(self, name: str, map_yaml: str | None = None) -> int:
        if name != 'base' and not self.base_ready():
            return 1
        if 'slam' in [c for c in SERVICES[name].conflicts if self.procs.running(c)]:
            warn = 'SLAM 맵핑을 끕니다 — 저장 안 한 지도는 사라집니다 (먼저 저장하려면 취소)'
            if self.ui.console.isatty():
                if not self.ui.confirm(f'{SERVICES[name].label}를 시작할까요?',
                                       body=[self.term.paint(warn, YELLOW)], default=True):
                    return 1
            else:
                print(self.term.paint('주의: ' + warn, YELLOW))
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
                    print(f'지도 없음: {args.map or "(최근 지도)"} — {self.procs.map_dir_txt} 확인 · `map list`', file=sys.stderr)
                    return 1
                map_yaml = str(path)
            return self.start(name, map_yaml)
        if args.action == 'stop':
            return self.stop(name)
        if args.action == 'restart':
            ok, msg = self.procs.restart(name)
            print(msg)
            return 0 if ok else 1
        if args.action == 'log':
            print(self.procs.tail(name, args.lines))
            return 0
        on = self.procs.running(name)
        print(f'{SERVICES[name].label}: {"실행 중" if on else "꺼짐"}')
        return 0 if on else 3

    def cmd_stop_all(self, args) -> int:
        self.send_velocity(0.0, 0.0)
        if hasattr(self.procs, 'stop_all'):   # 원격: 로봇의 stop-all이 pid로 확실히 끔
            ok, msg = self.procs.stop_all()
            print(msg)
            return 0 if ok else 1
        for name in reversed(list(SERVICES)):
            if self.procs.running(name):
                self.stop(name)
        print(self.procs.node_cleanup())   # 재시작 추적 노드도 같이 정리
        print('전부 종료')
        return 0

    # ------------------------------------------------------------- 개별 노드
    def _node_guard(self, name: str, force: bool) -> str | None:
        """빈 이름·핵심 노드 검사 — 문제없으면 None. 한 번 명령용."""
        from .nodes import CRITICAL, base
        name = base(name)
        if not name:
            return '노드 이름이 비었음'
        if name in CRITICAL and not force:
            return (f'{name}: 제어 핵심 노드 — 메뉴에서 확인 후 실행하거나 '
                    f'`--force` 필요 (예: run-cli node-restart {name} --force)')
        return None

    def cmd_node_kill(self, args) -> int:
        from .nodes import base
        err = self._node_guard(args.name, args.force)
        if err:
            print(err)
            return 2
        self.send_velocity(0.0, 0.0)
        ok, msg = self.procs.node_kill(base(args.name), force=args.force)
        print(msg)
        return 0 if ok else 1

    def cmd_node_restart(self, args) -> int:
        from .nodes import base
        err = self._node_guard(args.name, args.force)
        if err:
            print(err)
            return 2
        self.send_velocity(0.0, 0.0)
        ok, msg = self.procs.node_restart(base(args.name), force=args.force)
        print(msg)
        return 0 if ok else 1

    # ------------------------------------------------------------- 지도
    @staticmethod
    def default_map_name() -> str:
        return time.strftime('%Y%m%d_%H%M%S')

    def cmd_map(self, args) -> int:
        if args.action == 'list':
            maps = self.procs.maps()
            if not maps:
                print(f"지도 없음 ({self.procs.map_dir_txt})")
            for name, path in maps:   # 최신 먼저 (원격이면 로봇의 경로)
                print(f'{name:<24} {path}')
            return 0
        name = args.name or self.default_map_name()
        from .procs import IS_ROBOT
        if not IS_ROBOT:   # 로봇의 run-cli가 저장 (지도는 로봇에 — Nav가 거기서 씀)
            ok, msg = self.procs.save_map(name)
            print(msg)
            return 0 if ok else 1
        end = time.monotonic() + 5.0   # 막 켠 프로세스(원격 저장)는 latched /map을 받기까지 몇 초
        while self.state.map.age() is None and time.monotonic() < end:
            self._poll(0.2)
        if self.state.map.age() is None:
            print(self.term.paint('/map 수신 없음 — SLAM이 지도를 내고 있지 않습니다 (라이다 /scan이 있어야 지도가 생김)', RED))
            return 1
        if not self.procs.running('slam'):
            print(self.term.paint('주의: SLAM이 꺼져 있음 — /map을 내는 노드(map_server 등)가 있어야 저장됩니다', YELLOW))
        ok, msg = self.procs.save_map(name)
        print(msg)
        return 0 if ok else 1

    # ------------------------------------------------------------- 상태 요청 (cmd_manager가 판단 → state_manager가 알림)
    def request(self, key: str, value) -> bool:
        """/cmd_manual_<key> 발행 → /state_<key>가 그 값이 될 때까지 (최대 3초) 기다림."""
        pub, typ = self._req[key]
        if not self.wait_subscriber(pub, 'cmd_manager 없음 — 베이스(tb-only)가 떠 있는지 확인', 5.0):
            return False
        pub.publish(typ(data=value))
        end = time.monotonic() + 3.0
        while getattr(self.state, f'st_{key}') != value and time.monotonic() < end:
            self._poll(0.1)
        return getattr(self.state, f'st_{key}') == value

    def _wait_state(self) -> None:
        end = time.monotonic() + 2.0  # 새로 켠 CLI는 latched /state_control를 받기 전 — 잠깐 기다림
        while self.state.st_control is None and time.monotonic() < end:
            self._poll(0.1)

    def ensure_manual(self) -> bool:
        """수동 주행 전: MANUAL이 아니면 바꿀지 묻는다 (수동 명령은 MANUAL에서만 바퀴로 감). False = 취소."""
        self._wait_state()
        if self.state.st_control == 'MANUAL':
            return True
        if not sys.stdin.isatty():   # 스크립트에선 묻지 않고 거부 (입력 대기로 멈추지 않게)
            print(self.term.paint(f'거부: 제어 상태 {self.state.st_control} — 먼저 ./run-cli mode manual', RED))
            return False
        body = [f'지금 제어 상태: {self.state.st_control or "?"}',
                '수동 명령은 MANUAL에서만 바퀴로 갑니다.',
                'NAV_PLAN·SLAM_AUTO_MAPPING이면 바꾸는 순간 Nav2 주행(목표·자동 맵핑)은 바퀴로 안 갑니다.']
        if not self.ui.confirm('MANUAL로 바꿀까요?', default=True, body=body):
            print('취소 — 제어 상태 그대로')
            return False
        ok = self.request('control', 'MANUAL')
        print('제어 상태 → MANUAL' if ok else self.term.paint('MANUAL로 못 바꿈', RED))
        return ok

    def ensure_nav(self) -> None:
        """목표 보내기 전: NAV로 (Nav2 /cmd_vel은 NAV_PLAN·SLAM_AUTO_MAPPING에서만 바퀴로)."""
        self._wait_state()
        if self.state.st_control != 'NAV_PLAN' and self.request('control', 'NAV_PLAN'):
            print('제어 상태 → NAV_PLAN')

    def cmd_mode(self, args) -> int:
        value = args.value.upper().replace('-', '_')
        value = {'NAV': 'NAV_PLAN', 'AUTO': 'NAV_PLAN', 'AUTO_MAPPING': 'SLAM_AUTO_MAPPING'}.get(value, value)
        if value == 'SLAM_AUTO_MAPPING':   # SLAM·주행부부터 켜야 함
            return self.cmd_automap(argparse.Namespace(value='start'))
        ok = self.request('control', value)
        print(f'모드: {self.state.st_control}' if ok else self.term.paint('모드 변경 실패', RED))
        return 0 if ok else 1

    def cmd_route(self, args) -> int:
        """바퀴 명령 경로: direct = cmd_manager → opencr_driver · rl = PC RL 보정을 거침."""
        want = args.value in ('rl', 'on')
        if want and not self.rl_node_up():
            print(self.term.paint('RL 노드(rl_corrector)가 없음 — 켜면 바퀴가 바로 멈춥니다. 먼저 PC에서:', RED))
            print('  ros2 launch tb3_bringup rl-only.launch.py            # 보정 없이 통과 (연결 확인)')
            print('  ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=/경로/policy.onnx')
            print('  (ROS_DOMAIN_ID=25 · 자세히: ros2_ws/STATE_CONTROL.md 2장)')
            return 1
        ok = self.request('rl', want)
        if ok and self.state.st_rl:
            print('경로 → RL 경유 — PC에서 ros2 launch tb3_bringup rl-only.launch.py 가 떠 있어야 바퀴가 움직입니다')
        elif ok:
            print('경로 → 직결 (cmd_manager → opencr_driver)')
        else:
            print(self.term.paint('경로 변경 실패', RED))
        return 0 if ok else 1

    cmd_rl = cmd_route   # 예전 이름 (rl on|off)

    def rl_node_up(self, timeout: float = 3.0) -> bool:
        """rl-only의 rl_corrector가 그래프에 보이나 (막 켠 CLI는 탐색 시간을 조금 줌)."""
        end = time.monotonic() + timeout
        while True:
            if any(n == 'rl_corrector' for n, _ in self.node.get_node_names_and_namespaces()):
                return True
            if time.monotonic() >= end:
                return False
            self._poll(0.3)

    def cmd_control(self, args) -> int:
        return self.cmd_mode(args)

    def cmd_automap(self, args) -> int:
        """자동 맵핑: SLAM + SLAM 중 Nav를 켜고 제어 상태 SLAM_AUTO_MAPPING (explore_lite가 돌아다님 — explore_bridge가 켜고 끔)."""
        if args.value == 'stop':
            ok = self.request('control', 'MANUAL')
            print('자동 맵핑 중지 → MANUAL (지도 저장: ./run-cli map save 이름)' if ok else self.term.paint('중지 실패', RED))
            return 0 if ok else 1
        self.ui.step('0/4 베이스 확인 중')
        if not self.base_ready():
            return 1
        for i, name in enumerate(('slam', 'explore'), 1):
            if not self.procs.running(name):
                self.ui.step(f'{i}/4 {SERVICES[name].label} 켜는 중 (로봇에서 ~10초)')
                ok, msg = self.procs.start(name)
                print(msg)
                if not ok:
                    return 1
        self.ui.step('3/4 Nav2 활성화 대기')
        if not self.nav2_active(60.0):
            print(self.term.paint('Nav2 주행부가 활성화 안 됨 (60초) — ./run-cli explore log', RED))
            return 1
        # 멈춘 채 시작하면 SLAM 지도엔 스캔 1장뿐 → 로봇 바로 옆만 '빈 칸'(min_pass_through 2) → frontier가 다 닿을 수 없는 곳이라
        # explore_lite가 전부 포기하고 바로 끝남. 제자리 한 바퀴로 스캔을 여러 장 넣어 주변을 먼저 채운다.
        self.ui.step('4/4 제자리에서 한 바퀴 돌며 주변 찍기')
        if not self.request('control', 'MANUAL'):
            return 1
        self.spin_once()
        ok = self.request('control', 'SLAM_AUTO_MAPPING')
        print('자동 맵핑 시작 — 진행은 상태바 "탐색" 줄 · 끝나면 MANUAL로 돌아옴 · 중지: ./run-cli automap stop'
              if ok else self.term.paint('SLAM_AUTO_MAPPING으로 못 바꿈', RED))
        return 0 if ok else 1

    def cmd_estop(self, args) -> int:
        on = args.value == 'on'
        if on:
            self.send_velocity(0.0, 0.0)
        ok = self.request('estop', on)
        print(f'비상정지: {"ON" if self.state.st_estop else "OFF"}' if ok else self.term.paint('비상정지 변경 실패', RED))
        return 0 if ok else 1

    # ------------------------------------------------------------- 주행
    def wait_subscriber(self, pub, missing: str, timeout: float = 10.0) -> bool:
        """방금 만든 발행자는 상대와 연결(discovery)되기 전 — 구독자가 생길 때까지 기다림. 없으면 missing 출력."""
        end = time.monotonic() + timeout
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            self._poll(0.2)
        if pub.get_subscription_count() == 0:
            print(self.term.paint(missing, RED))
            return False
        return True

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
        if not self.ensure_manual():
            return 1
        speed, ang = min(args.speed, MAX_LIN), min(args.angular, MAX_ANG)
        v = {'forward': speed, 'back': -speed}.get(args.direction, 0.0)
        w = {'left': ang, 'right': -ang}.get(args.direction, 0.0)
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
        if not self.ensure_manual():
            return 1
        from .dialogs import Dialog, KeyReader
        move = (0, 0)   # (전후, 회전) 방향 — 속도는 lin_speed·ang_speed를 곱해서 (속도를 바꾸면 달리는 중에도 바로 반영)
        lin_speed, ang_speed = min(args.linear, MAX_LIN), min(args.angular, MAX_ANG)
        last_send = 0.0
        last_key, key_count = '-', 0
        box = Dialog(self.ui, f'텔레오퍼 → {CMD_VEL}', [])   # 공용 박스 — console로 그려 캡처를 우회
        alias = {'up': 'w', 'down': 'x', 'left': 'a', 'right': 'd', ' ': 's'}
        try:
            with KeyReader(self.term) as keys:
                while rclpy.ok():
                    linear = move[0] * lin_speed
                    angular = move[1] * (lin_speed / DIAG_RADIUS if move[0] else ang_speed)
                    key_txt = f'키 {last_key} ×{key_count}' if key_count else '키 입력 대기 중'
                    box.lines = [box.title,
                                f'명령 v={linear:+.2f} m/s ω={angular:+.2f} rad/s',
                                f'odom {self.state.odom_txt()}',
                                f'scan {self.state.scan_txt()}',
                                f'map  {self.state.map_txt()}',
                                f'{key_txt} · 설정 v {lin_speed:.2f} m/s · ω {ang_speed:.2f} rad/s',
                                self.term.paint('w/x/a/d·방향키 이동 · q/e/z/c 사선 · s/space 정지', DIM),
                                self.term.paint('-/= 속도± · b 삑 · Esc 종료', DIM)]
                    box.draw('key')
                    now = time.monotonic()
                    if now - last_send >= 0.1:
                        self.send_velocity(linear, angular)
                        last_send = now
                    key = keys.read(0.1)
                    self._poll(0.01)
                    if key is None:
                        continue
                    if key in ('esc', 'eof', '\x03'):
                        break
                    key = alias.get(key, key)
                    key_count += 1
                    last_key = 'space' if key == 's' else key
                    if key in TELEOP_MOVES:
                        move = TELEOP_MOVES[key]
                    elif key == 'b':
                        self._beep_pub.publish(Empty())
                    elif key in ('=', '+'):   # 상한 = Waffle Pi 사양
                        lin_speed, ang_speed = min(lin_speed + 0.05, MAX_LIN), min(ang_speed + 0.3, MAX_ANG)
                    elif key in ('-', '_'):
                        lin_speed, ang_speed = max(lin_speed - 0.05, 0.05), max(ang_speed - 0.3, 0.2)
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
        if not self.wait_subscriber(self._init_pub, '/initialpose 구독자 없음 — Nav(amcl)가 떠 있는지 확인'):
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

    def nav2_active(self, timeout: float) -> bool:
        """Nav2 lifecycle_manager가 노드를 다 활성화했나 — action 서버가 '있기만' 하면 목표를 REJECTED함."""
        from std_srvs.srv import Trigger
        cli = self.node.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        start = time.monotonic()
        try:
            while time.monotonic() - start < timeout:
                self.ui.progress(f'Nav2 켜지는 중 … {time.monotonic() - start:.0f}/{timeout:.0f}초 (Pi에서 보통 20~40초)')
                if cli.service_is_ready():
                    fut = cli.call_async(Trigger.Request())
                    end = time.monotonic() + 2.0
                    while not fut.done() and time.monotonic() < end:
                        self._poll(0.1)
                    if fut.done() and fut.result() is not None and fut.result().success:
                        return True
                self._poll(0.5)
            return False
        finally:
            self.node.destroy_client(cli)

    def spin_once(self, speed: float = 0.5) -> None:
        """제자리 한 바퀴 (반시계, speed rad/s) — 끝나거나 중단돼도 0을 보냄."""
        duration = 2.0 * math.pi / speed + 0.5
        start = time.monotonic()
        try:
            while time.monotonic() - start < duration:
                left = duration - (time.monotonic() - start)
                self.ui.progress(f'회전 중 … {left:.0f}초 남음 (주변을 비우세요)')
                self.send_velocity(0.0, speed)
                self._poll(0.1)
        finally:
            for _ in range(3):
                self.send_velocity(0.0, 0.0)
                self._poll(0.05)

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
        self.ensure_nav()
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
        if not self.wait_subscriber(self._reboot_pub, DRIVER_MISSING):
            return 1
        self.send_velocity(0.0, 0.0)
        self._reboot_pub.publish(Empty())
        print('모터 재부팅 요청 (R) — 바퀴가 잠깐 힘이 빠졌다가 돌아옵니다')
        self._poll(3.0)  # 재부팅(바퀴당 ~1초) 후 진단 갱신
        print(f'로봇: {self.state.robot_txt()}')
        return 0 if self.state.robot_parts()[0] == 0 else 1

    def cmd_beep(self, args) -> int:
        """OpenCR 부저 — 인자 없으면 기본 삑, `beep <Hz> <ms>`면 높이·길이 지정."""
        vals = getattr(args, 'value', None) or []
        if len(vals) == 2:
            try:
                hz, ms = max(100, min(8000, int(vals[0]))), max(10, min(2000, int(vals[1])))
            except ValueError:
                print('숫자로: beep <Hz 100~8000> <ms 10~2000> (예: beep 4000 300)')
                return 1
            if not self.wait_subscriber(self._beep_tone_pub, 'cmd_manager 없음 — 베이스(tb-only)가 떠 있는지 확인'):
                return 1
            from std_msgs.msg import Int32MultiArray
            self._beep_tone_pub.publish(Int32MultiArray(data=[hz, ms]))
            self._poll(0.2)
            print(f'삑 {hz}Hz {ms}ms (B,{hz},{ms})')
            return 0
        if not self.wait_subscriber(self._beep_pub, 'cmd_manager 없음 — 베이스(tb-only)가 떠 있는지 확인'):
            return 1
        self._beep_pub.publish(Empty())
        self._poll(0.2)
        print('삑 (B)')
        return 0

    def say_text(self, text: str) -> int:
        """타이핑한 한/영을 로봇에 문자열로 — 변환·재생은 cmd_manager(/cmd_say)."""
        from std_msgs.msg import String
        if not self._say_pub:
            self._say_pub = self.node.create_publisher(String, '/cmd_say', 10)
        if not self.wait_subscriber(self._say_pub, 'cmd_manager 없음 — 베이스(tb-only)가 떠 있는지 확인'):
            return 1
        self._say_pub.publish(String(data=text))
        print(f'삑 전송 ("{text}") — 소리는 로봇 cmd_manager가 만듦')
        return 0

    def cmd_say(self, args) -> int:
        """타이핑삑 한 번 명령 — `say 안녕 hello`."""
        text = ' '.join(getattr(args, 'text', None) or []).strip()
        if not text:
            print('사용법: say <글> (예: say 안녕 hello)')
            return 1
        return self.say_text(text)

    # ------------------------------------------------------------- 노드
    def node_names(self) -> list[str]:
        self._poll(0.5)
        names = self.node.get_node_names_and_namespaces()
        out = sorted(f'{ns.rstrip("/")}/{n}' for n, ns in names if n != self.node.get_name())
        now = time.monotonic()
        for n in out:   # 그래프에서 본 시각 기록 (사라진 노드의 마지막 확인용)
            self.state.nodes_seen[n] = now
        for n in [k for k, t in self.state.nodes_seen.items() if now - t > 3600]:
            del self.state.nodes_seen[n]
        return out

    def node_rows(self) -> list[tuple[str, bool, float]]:
        """(이름, 현재 보임, 마지막 확인 경과초). 사라진 건 10분까지만 표시."""
        names = self.node_names()
        now = time.monotonic()
        alive = set(names)
        rows = [(n, True, now - self.state.nodes_seen.get(n, now)) for n in names]
        rows += [(n, False, now - t) for n, t in sorted(self.state.nodes_seen.items())
                 if n not in alive and now - t < 600]
        return rows

    def cmd_nodes(self, args) -> int:
        names = self.node_names()
        print('\n'.join(names) if names else '노드 없음 (같은 ROS_DOMAIN_ID인지 확인)')
        return 0
