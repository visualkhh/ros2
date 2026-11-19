#!/usr/bin/env python3
"""로봇 운영 CLI — 베이스 · SLAM 맵핑 · 지도 저장 · Nav 주행 · 텔레오퍼를 한 곳에서.

로봇(Pi)에서 실행한다. SLAM·Nav는 이 tool이 `ros2 launch`로 직접 띄우고 끈다
지도가 만들어지는 모습은 PC에서 tools/run-rviz로 본다.

  ./tools/run-cli                  # 대화형 메뉴
  ./tools/run-cli status           # 한 번만 출력 (자동화용)
  ./tools/run-cli slam start
  ./tools/run-cli teleop
  ./tools/run-cli map save office
  ./tools/run-cli nav start office
  ./tools/run-cli initpose 0 0 0
  ./tools/run-cli goal 1.0 0.5 90

구조 (OOP 분리):
  op/terminal.py  터미널 원자 연산
  op/keys.py      키 읽기 (cbreak · 방향키 · UTF-8)
  op/component.py Component — 화면 부품 최상위 부모 (focusable · activate)
  op/window/      Window — 상태바·메뉴 막대·중앙·힌트 조합 · 키 루프 · 신호
  op/bars/menu/   MenuBar → LeftSideMenuBar · ChoiceItem
  op/bars/state/  State(칸) · StateBar · TopStateBar
  op/dialog/      Dialog 최상위 부모 + Input·Key·Pager·Busy·Teleop
  op/ui.py        Ui — 터미널 모드 · 창 진입점 · ui.window
  op/state.py     StateStore — ROS 상태 보관
  op/procs.py     Procs — launch 프로세스 · 지도 파일
  op/bus.py       Bus — 발행·액션·명령 실행
  op/pages/       Page → Menu → 각 메뉴 화면
  이 파일         OperatorTool(Node): 배선 + argparse + main
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray

from op.bus import Bus
from op.defs import STAY
from op.procs import SERVICES, make_procs
from op.state import StateStore
from op.terminal import Terminal
from op.ui import Ui

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class OperatorTool(Node):
    """배선 전용 노드 — 상태·화면·버스는 op/ 패키지가 맡는다."""

    def __init__(self):
        super().__init__('operator_tool')
        self.term = Terminal()
        self.state = StateStore()
        self.tf_buffer = Buffer()                    # SLAM 중 map→base 자세 (amcl 없을 때)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.procs = make_procs(self)   # 로봇이 아니면 RemoteProcs (SSH + 노드 그래프)
        self.ui = Ui(self.term, self.state, self.procs)
        self.bus = Bus(self, self.state, self.ui, self.term)
        self.ui.bus = self.bus
        self.drive_cfg = {'linear': 0.15, 'angular': 0.8, 'duration': 1.0}
        self._collision_at = 0.0
        # 새 CLI/TUI도 마지막 저장된 토글을 바로 표시 (실제 적용은 Nav/노드 시작 때)
        try:
            saved_enable = self.procs.saved_enable()
        except Exception:
            saved_enable = None
        if saved_enable is not None:
            self.state.rl_enabled = saved_enable
            self.state.rl_enabled_at = time.monotonic()
        try:
            saved_collision = self.procs.saved_collision_enabled()
        except Exception:
            saved_collision = None
        if saved_collision is not None:
            self.state.collision_enabled = saved_collision
            self.state.collision_override = saved_collision
        # 화면 틀(Window) 신호 — ROS 폴링은 'tick', 이름으로 화면 열기는 'navigate'
        self.ui.window.connect('tick', self._on_tick)
        self.ui.window.connect('navigate', self._on_navigate)
        self._wire()

    def poll(self, seconds: float = 0.2):
        """콜백을 잠깐 돌려 상태를 갱신한다."""
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.01, end - time.monotonic())))

    def _on_tick(self, seconds: float):
        if not rclpy.ok():   # 종료 신호 — 메뉴 루프를 끝낸다
            raise SystemExit(0)
        self.poll(seconds)
        now = time.monotonic()
        if now - self._collision_at > 5.0:   # collision_monitor enabled 파라미터 — 상단 칸용 캐시
            self._collision_at = now
            try:
                self.bus.poll_collision_enabled()
            except Exception:
                pass

    def _on_navigate(self, name: str, params: dict):
        page = self.screen(name, **params)
        return page.show() if page is not None else STAY

    def _wire(self):
        self.create_subscription(Odometry, '/odom', self.state.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.state.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.state.on_map, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.state.on_amcl, LATCHED)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.state.on_diagnostics, 10)
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, '/joint_states', self.state.on_joint_states, 10)
        from std_msgs.msg import Bool, String
        for key, typ in (('control', String), ('estop', Bool), ('operation', String), ('map', String)):
            self.create_subscription(typ, f'/state_{key}',
                                     lambda m, k=key: self.state.on_state(k, m.data), LATCHED)
        self.create_subscription(String, '/explore_status', lambda m: setattr(self.state, 'explore_status', m.data), 10)
        try:
            from nav2_msgs.msg import CollisionMonitorState
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                     self.state.on_collision_state, 10)
        except Exception:
            pass   # fake/docker 등 nav2_msgs가 없는 환경
        # 노드별 추가 상태·기능 (예: controller_server RL/MPPI)
        from op.pages.nodes import wire_all
        wire_all(self)

    def screen(self, name: str, **params):
        """화면 인스턴스. params = 화면에 넘기는 값 (웹 쿼리처럼 — 예: map 화면의 mode·source)."""
        from op import pages
        if name == 'map':
            return pages.MapPage(self, **params)
        return {
            '1': pages.StatusMenu, '2': pages.BaseMenu, '3': pages.SlamMenu, '4': pages.NavMenu,
            '5': pages.DriveMenu, '6': pages.MapMenu, '7': pages.ExploreMenu, '8': pages.NodesMenu,
            't': pages.TopicBagMenu, 'collision': pages.CollisionMenu, 'diagnostics': pages.DiagnosticsMenu,
            'mode': pages.ModeMenu, 'localization': pages.LocalizationMenu, 'map-change': pages.MapChangePage, '9': pages.StopAllPage, '0': pages.BeepMenu,
        }[name](self)

    # ------------------------------------------------------------- CLI 진입
    def cmd_node_status(self, args) -> int:
        self.poll(1.0)
        print('\n'.join(self.bus.node_status_lines(args.name)))
        return 0

    def cmd_menu(self, args) -> int:
        from op.pages import MainMenu
        return int(MainMenu(self).show() or 0)

    def __getattr__(self, name: str):
        # cmd_* 한 번 명령은 Bus에 위임
        if name.startswith('cmd_'):
            try:
                bus = object.__getattribute__(self, 'bus')
            except AttributeError:
                raise AttributeError(name) from None
            if name[4:] in SERVICES:
                return bus.cmd_svc
            return getattr(bus, name.replace('-', '_'))
        raise AttributeError(name)


def _positive(raw: str) -> float:
    v = float(raw)
    if v <= 0:
        raise argparse.ArgumentTypeError('0보다 커야 합니다')
    return v


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='run-cli',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''맵핑 순서:
  ./run-cli base start          # 바퀴·odom·라이다
  ./run-cli slam start          # 지도 만들기 시작 (PC: ./run-rviz slam 으로 보기)
  ./run-cli teleop              # 천천히 돌아다니기
  ./run-cli map save office     # ros2_ws/map/office/map.yaml
  ./run-cli slam stop

주행 순서:
  ./run-cli nav start office    # (PC: ./run-rviz nav)
  ./run-cli initpose 0 0 0
  ./run-cli goal 1.0 0.5 90
''')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('menu', help='대화형 메뉴 (인자 없이 실행했을 때 기본값)')
    sub.add_parser('status', help='프로세스·토픽 상태')

    st = sub.add_parser('start', help='베이스 + Nav/RL 시작 (기본: 최근 지도)')
    st.add_argument('target', nargs='?', default='all',
                    help='base | nav | all (생략=all) · 또는 지도 이름')
    st.add_argument('map', nargs='?', help='Nav 지도 이름 (target=nav/all)')
    st.add_argument('--model', metavar='ONNX',
                    help='RL 정책 .onnx (생략하면 설치된 share 모델 자동 사용)')

    for name, svc in SERVICES.items():
        p = sub.add_parser(name, help=f'{svc.label} — {svc.desc}')
        p.add_argument('action', type=str.lower, choices=('start', 'stop', 'restart', 'switch', 'status', 'log'))
        if name == 'nav':
            p.add_argument('map', nargs='?', help='지도 이름·폴더·yaml (기본: 가장 최근 지도)')
            p.add_argument('--model', metavar='ONNX',
                           help='RL 정책 .onnx — 주면 FollowPath.enable=true로 시작 '
                                '(PC 경로면 .onnx.data와 함께 로봇으로 복사)')
        p.add_argument('--lines', type=int, default=60, help='log 줄 수')

    m = sub.add_parser('map', help='지도 파일 목록·저장')
    m.add_argument('action', type=str.lower, choices=('list', 'save'))
    m.add_argument('name', nargs='?', help='save 이름 (기본: 날짜_시각)')
    m.add_argument('--no-switch', action='store_true',
                   help='save 후 저장한 지도로 LOCALIZATION 전환을 묻지 않음')

    t = sub.add_parser('teleop', help='키보드 수동 주행')
    t.add_argument('--linear', type=_positive, default=0.15, help='선속도 [m/s] (기본 0.15 · -/=로 ±, 최대 0.26)')
    t.add_argument('--angular', type=_positive, default=0.8, help='각속도 [rad/s] (기본 0.8 · 최대 1.82)')

    d = sub.add_parser('drive', help='정해진 시간만큼 이동')
    d.add_argument('direction', type=str.lower, choices=('forward', 'back', 'left', 'right', 'stop'))
    d.add_argument('--speed', type=_positive, default=0.15, help='최대 0.26')
    d.add_argument('--angular', type=_positive, default=0.8, help='최대 1.82')
    d.add_argument('--duration', type=_positive, default=1.0)

    ip = sub.add_parser('initpose', help='amcl 초기 위치 (map 좌표)')
    ip.add_argument('x', type=float)
    ip.add_argument('y', type=float)
    ip.add_argument('yaw', type=float, nargs='?', default=0.0, help='각도 [°]')

    g = sub.add_parser('goal', help='Nav2 목표 (map 좌표) — 도착까지 기다림')
    g.add_argument('x', type=float, nargs='?')
    g.add_argument('y', type=float, nargs='?')
    g.add_argument('yaw', type=float, nargs='?', default=0.0, help='각도 [°]')
    g.add_argument('--no-wait', action='store_true', help='보내고 바로 끝냄')
    g.add_argument('--cancel', action='store_true', help='이 CLI가 보낸 목표 취소')

    sub.add_parser('nodes', help='보이는 ROS 노드 목록')
    p = sub.add_parser('node-status', help='노드 하나의 상태·발행/구독·진단 (예: /controller_server)')
    p.add_argument('name', help='노드 이름 (예: /controller_server)')

    bag = sub.add_parser('bag', help='rosbag2(MCAP) 녹화 — bag start | stop | status (메뉴 t)')
    bag.add_argument('action', type=str.lower, choices=('start', 'stop', 'status'))
    bag.add_argument('--label', '-l', default=None, help='start: 폴더 이름에 붙일 라벨 (예: manual)')
    bag.add_argument('--topics', nargs='*', default=None, help='start: 이 토픽만 (생략하면 진단 프리셋)')
    bag.add_argument('--regex', default=None, help='start: 토픽 정규식 (프리셋 대신)')
    bag.add_argument('--no-sys', action='store_true', help='start: 부하(sys.csv) 기록 끔')
    for cmd, help_ in (('node-kill', '개별 노드 중지 — OS 프로세스 kill (로봇에서 실행 · PC는 SSH 경유)'),
                       ('node-restart', '개별 노드 재시작 — kill 후 같은 명령으로 재실행 · 추적됨')):
        p = sub.add_parser(cmd, help=help_)
        p.add_argument('name', help='노드 이름 (예: scan_resample)')
        p.add_argument('--force', action='store_true',
                      help='제어 핵심 노드(state/cmd/opencr)도 강제 (메뉴에서는 확인창으로 대체)')
    for name, choices, help_ in (('control', ('manual', 'nav-plan', 'slam-auto-mapping'),
                                  '제어 상태: manual=텔레오퍼 · nav-plan=Nav2 목표 · slam-auto-mapping=자동 맵핑 (/cmd_manual_control)'),
                                 ('mode', ('manual', 'nav-plan', 'nav', 'slam-auto-mapping', 'auto'), '= control (예전 이름)'),
                                 ('automap', ('start', 'stop'), '자동 맵핑: SLAM·SLAM 중 Nav 켜고 SLAM_AUTO_MAPPING / 중지(MANUAL)'),
                                 ('estop', ('on', 'off'), '비상정지 (/cmd_manual_estop)')):
        p = sub.add_parser(name, help=help_)
        p.add_argument('value', type=str.lower, choices=choices)
    p = sub.add_parser('collision', help='충돌 감시 (collision_monitor) 켜기/끄기/상태')
    p.add_argument('state', type=str.lower, choices=('on', 'off', 'status'), nargs='?', default='status')
    p = sub.add_parser('fallback', help='RL 바이패스 controller 변경 (mppi|dwb|rpp|rotation_shim) — 설정 저장·빌드·Nav 재시작')
    p.add_argument('name', type=str.lower, choices=('mppi', 'dwb', 'rotation_shim'))
    p = sub.add_parser('fallback-enable', help='FollowPath.enable 런타임 값 저장 (on|off|status|clear) — 다음 재시작에 적용')
    p.add_argument('value', type=str.lower, choices=('on', 'off', 'status', 'clear', 'apply'))
    sub.add_parser('reboot-motors', help='모터 에러 해제 — 두 바퀴 재부팅 (/opencr/reboot)')
    p = sub.add_parser('beep', help='OpenCR 부저 — 삑 또는 beep <Hz 100~8000> <ms 10~2000>')
    p.add_argument('value', nargs='*', help='예: 4000 300')
    p = sub.add_parser('say', help='타이핑삑 — 한/영 문자열을 자모별 주파수 삑으로 (/cmd_say)')
    p.add_argument('text', nargs='+', help='예: say 안녕 hello')
    sub.add_parser('stop', help='띄운 것 전부 종료 (바퀴 정지 먼저)')
    sub.add_parser('stop-all', help=argparse.SUPPRESS)   # 예전 이름 호환
    sub.add_parser('restart-all', help='켜져 있던 서비스만 같은 순서로 전부 재시작 (nav는 같은 지도로)')

    b = sub.add_parser('build', help='colcon build (로봇 Pi · PC는 SSH 경유) — --full은 setup/pi/build.sh')
    b.add_argument('--full', action='store_true',
                   help='setup/pi/build.sh: reset + vcs import + rosdep + patch + build (deps.repos·patches 변경 후)')
    b.add_argument('args', nargs=argparse.REMAINDER,
                   help='colcon build에 그대로 전달 (예: --packages-select tb3_bringup)')
    return parser


def split_build(argv: list[str]) -> argparse.Namespace:
    """build 인자 — --full만 옵션으로 빼고 나머지는 colcon에 순서 그대로 넘긴다."""
    rest, full = list(argv[1:]), False
    while rest and rest[0] == '--full':
        full = True
        rest = rest[1:]
    return argparse.Namespace(command='build', full=full, args=rest)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        argv = ['menu'] if sys.stdin.isatty() else ['status']
    parser = build_parser()
    if argv and argv[0] == 'build':
        # build는 colcon 옵션(--packages-select 등)을 순서 그대로 받는다.
        # argparse REMAINDER는 옵션을 먼저 만나면 순서를 보존하지 못해서 직접 자른다.
        if (argv[1:2] in (['-h'], ['--help'])
                or (argv[1:2] == ['--full'] and argv[2:3] in (['-h'], ['--help']))):
            parser.parse_args(['build', '--help'])
        args = split_build(argv)
    else:
        args = parser.parse_args(argv)
    rclpy.init(args=[])
    node = OperatorTool()
    try:
        method = getattr(node, f'cmd_{args.command.replace("-", "_")}')
        return int(method(args) or 0)
    except KeyboardInterrupt:
        return 130
    finally:
        if args.command == 'menu':   # TUI가 켠 녹화는 TUI와 같이 끝 (q · Ctrl+C · SSH 끊김)
            try:
                from op import bagrec
                bagrec.stop_mine()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
