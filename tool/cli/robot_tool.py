#!/usr/bin/env python3
"""로봇 운영 CLI — 베이스 · SLAM 맵핑 · 지도 저장 · Nav 주행 · 텔레오퍼를 한 곳에서.

로봇(Pi)에서 실행한다. SLAM·Nav는 이 tool이 `ros2 launch`로 직접 띄우고 끈다
(state_manager 같은 상주 노드 없음). 지도가 만들어지는 모습은 PC에서 tool/run-rviz로 본다.

  ./tool/run-cli                  # 대화형 메뉴
  ./tool/run-cli status           # 한 번만 출력 (자동화용)
  ./tool/run-cli slam start
  ./tool/run-cli teleop
  ./tool/run-cli map save office
  ./tool/run-cli nav start office
  ./tool/run-cli initpose 0 0 0
  ./tool/run-cli goal 1.0 0.5 90

구조 (OOP 분리):
  op/terminal.py  터미널 원자 연산
  op/dialogs.py   키 읽기 · 다이얼로그 · 메뉴 루프
  op/ui.py        화면 전부 (프레임 + 상태바)
  op/state.py     StateStore — ROS 상태 보관
  op/procs.py     Procs — launch 프로세스 · 지도 파일
  op/bus.py       Bus — 발행·액션·명령 실행
  op/pages/       Page → Menu → 각 메뉴 화면
  이 파일         RobotTool(Node): 배선 + argparse + main
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
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray

from op.bus import Bus
from op.procs import SERVICES, make_procs
from op.state import StateStore
from op.terminal import Terminal
from op.ui import Ui

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class RobotTool(Node):
    """배선 전용 노드 — 상태·화면·버스는 op/ 패키지가 맡는다."""

    def __init__(self):
        super().__init__('robot_tool')
        self.term = Terminal()
        self.state = StateStore()
        self.procs = make_procs(self)   # 로봇이 아니면 RemoteProcs (SSH + 노드 그래프)
        self.ui = Ui(self.term, self.state, self.procs, self.poll)
        self.bus = Bus(self, self.state, self.ui, self.term)
        self.drive_cfg = {'linear': 0.15, 'angular': 0.8, 'duration': 1.0}
        self._wire()

    def poll(self, seconds: float = 0.2):
        """콜백을 잠깐 돌려 상태를 갱신한다."""
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.01, end - time.monotonic())))

    def _wire(self):
        self.create_subscription(Odometry, '/odom', self.state.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.state.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.state.on_map, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.state.on_amcl, LATCHED)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.state.on_diagnostics, 10)
        from std_msgs.msg import Bool, String
        for key, typ in (('control', String), ('rl', Bool), ('estop', Bool), ('operation', String), ('map', String)):
            self.create_subscription(typ, f'/state_{key}',
                                     lambda m, k=key: setattr(self.state, f'st_{k}', m.data), LATCHED)
        self.create_subscription(String, '/explore_status', lambda m: setattr(self.state, 'explore_status', m.data), 10)
        from geometry_msgs.msg import Twist
        self.create_subscription(Twist, '/cmd_vel_rl_out', lambda _m: self.state.rl_out.tick(), 10)

    def screen(self, name: str):
        from op import pages
        return {
            '1': pages.StatusMenu, '2': pages.BaseMenu, '3': pages.SlamMenu, '4': pages.NavMenu,
            '5': pages.DriveMenu, '6': pages.MapMenu, '7': pages.ExploreMenu, '8': pages.NodesMenu,
            '9': pages.StopAllPage,
        }[name](self)

    # ------------------------------------------------------------- CLI 진입
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

    for name, svc in SERVICES.items():
        p = sub.add_parser(name, help=f'{svc.label} — {svc.desc}')
        p.add_argument('action', type=str.lower, choices=('start', 'stop', 'restart', 'status', 'log'))
        if name == 'nav':
            p.add_argument('map', nargs='?', help='지도 이름·폴더·yaml (기본: 가장 최근 지도)')
        p.add_argument('--lines', type=int, default=60, help='log 줄 수')

    m = sub.add_parser('map', help='지도 파일 목록·저장')
    m.add_argument('action', type=str.lower, choices=('list', 'save'))
    m.add_argument('name', nargs='?', help='save 이름 (기본: 날짜_시각)')

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
    for name, choices, help_ in (('control', ('manual', 'nav-plan', 'slam-auto-mapping'),
                                  '제어 상태: manual=텔레오퍼 · nav-plan=Nav2 목표 · slam-auto-mapping=자동 맵핑 (/cmd_manual_control)'),
                                 ('mode', ('manual', 'nav-plan', 'nav', 'slam-auto-mapping', 'auto'), '= control (예전 이름)'),
                                 ('route', ('direct', 'rl'), '바퀴 명령 경로: direct=바로 opencr_driver · rl=PC RL 보정 경유'),
                                 ('rl', ('on', 'off'), '= route rl / route direct (예전 이름)'),
                                 ('automap', ('start', 'stop'), '자동 맵핑: SLAM·SLAM 중 Nav 켜고 SLAM_AUTO_MAPPING / 중지(MANUAL)'),
                                 ('estop', ('on', 'off'), '비상정지 (/cmd_manual_estop)')):
        p = sub.add_parser(name, help=help_)
        p.add_argument('value', type=str.lower, choices=choices)
    sub.add_parser('reboot-motors', help='모터 에러 해제 — 두 바퀴 재부팅 (/opencr/reboot)')
    p = sub.add_parser('beep', help='OpenCR 부저 — 삑 또는 beep <Hz 100~8000> <ms 10~2000>')
    p.add_argument('value', nargs='*', help='예: 4000 300')
    sub.add_parser('stop-all', help='띄운 것 전부 종료 (바퀴 정지 먼저)')
    return parser


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        argv = ['menu'] if sys.stdin.isatty() else ['status']
    args = build_parser().parse_args(argv)
    rclpy.init(args=[])
    node = RobotTool()
    try:
        method = getattr(node, f'cmd_{args.command.replace("-", "_")}')
        return int(method(args) or 0)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
