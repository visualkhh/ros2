"""프로세스 메뉴 — 베이스 · SLAM 맵핑 · Nav · SLAM 중 Nav (start/stop/재시작/로그 + 전용 항목)."""
from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import subprocess

from ..defs import BACK, STAY
from ..procs import RUN_DIR, SERVICES, TOOL_DIR
from ..terminal import DIM, GREEN, YELLOW
from .base import Menu, MenuItem, Page
from .drive import TeleopItem


def _xyyaw(raw: str) -> str | None:
    parts = raw.replace(',', ' ').split()
    if len(parts) not in (2, 3):
        return '형식: x y [yaw°]  예) 1.0 0.5 90'
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return f'숫자가 아님: {raw!r}'
    if not all(math.isfinite(v) for v in vals):
        return '유한한 숫자가 아님'
    return None


def _parse_xyyaw(raw: str) -> tuple[float, float, float]:
    vals = [float(p) for p in raw.replace(',', ' ').split()]
    return vals[0], vals[1], vals[2] if len(vals) > 2 else 0.0


# ---------------------------------------------------------------- 공용 항목
class StartItem(MenuItem):
    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self.name = name

    def label(self):
        return '시작'

    def visible(self):
        return not self.bus.procs.running(self.name)

    def run(self):
        self.bus.start(self.name)
        return STAY


class StopItem(MenuItem):
    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self.name = name

    def label(self):
        return '정지'

    def visible(self):
        return self.bus.procs.running(self.name)

    def run(self):
        svc = SERVICES[self.name]
        if not self.ui.confirm(f'{svc.label}를 끌까요?', default=True):
            return STAY
        if self.name in ('nav', 'explore'):
            self.bus.send_velocity(0.0, 0.0)
        self.bus.stop(self.name)
        return STAY


class RestartItem(MenuItem):
    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self.name = name

    def label(self):
        return '재시작'

    def visible(self):
        return self.bus.procs.running(self.name)

    def run(self):
        svc = SERVICES[self.name]
        body = [svc.desc]
        if self.name == 'slam':
            body = [self.term.paint('재시작하면 지금까지 만든 지도가 사라집니다 (먼저 저장)', YELLOW)]
        if not self.ui.confirm(f'{svc.label}를 재시작할까요?', body=body):
            return STAY
        print(self.bus.procs.restart(self.name)[1])
        return STAY


class LogItem(MenuItem):
    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self.name = name

    def label(self):
        return '로그 (마지막 200줄)'

    def run(self):
        self.ui.pager(f'{SERVICES[self.name].label} 로그 — {self.bus.procs.logfile(self.name)}',
                      self.bus.procs.tail(self.name, 200))
        return STAY


class ServiceMenu(Menu):
    """start/stop/재시작/로그 + extra()."""

    name = ''

    @property
    def title(self):
        return SERVICES[self.name].label

    def extra(self) -> list[MenuItem]:
        return []

    def items(self):
        return [StartItem(self.app, '1', self.name), StopItem(self.app, '1', self.name),
                RestartItem(self.app, '2', self.name), *self.extra(), LogItem(self.app, 'l', self.name)]

    def center(self):
        svc = SERVICES[self.name]
        on = self.bus.procs.running(self.name)
        lines = [f'[{svc.label}]', f' {svc.desc}',
                 ' 상태: ' + (self.term.paint('실행 중', GREEN) if on else self.term.paint('꺼짐', DIM))]
        if on:
            lines.append(f' {self.bus.service_rows()[list(SERVICES).index(self.name)][3]}')
        lines += [''] + self.bus.status_lines()[len(SERVICES) + 2:]
        return lines + [''] + self.hint()

    def hint(self) -> list[str]:
        return []


class BaseMenu(ServiceMenu):
    name = 'base'

    def hint(self):
        from ..procs import BASE_CMD
        return [self.term.paint(f' 명령: {BASE_CMD}', DIM),
                self.term.paint(' 실기 드라이버는 TOOL_BASE_CMD 로 바꿈', DIM)]


class RvizItem(MenuItem):
    """이 컴퓨터에 RViz 창 띄우기 (tool/run-rviz <preset>) — CLI는 그대로 쓰면서 따로 창으로."""

    key = 'v'

    def __init__(self, app, preset: str):
        super().__init__(app)
        self.preset = preset

    def label(self):
        return f'RViz로 보기 ({self.preset})'

    def run(self):
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            print('화면이 없는 터미널입니다 (SSH 등) — 화면이 있는 PC에서 ./run-rviz ' + self.preset)
            return STAY
        if shutil.which('pgrep') and subprocess.run(['pgrep', '-x', 'rviz2'], capture_output=True).returncode == 0:
            print('RViz가 이미 떠 있습니다 — 다른 화면으로 바꾸려면 ./run-rviz restart ' + self.preset)
            return STAY
        log = open(RUN_DIR / 'rviz.log', 'w')
        subprocess.Popen([str(TOOL_DIR / 'run-rviz'), self.preset], stdin=subprocess.DEVNULL,
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f'RViz 여는 중 ({self.preset}) — 창이 뜨는 데 몇 초 · 로그 {RUN_DIR / "rviz.log"}')
        return STAY


# ---------------------------------------------------------------- SLAM
class SaveMapItem(MenuItem):
    key = '3'

    def label(self):
        return '지도 저장'

    def run(self):
        default = self.bus.default_map_name()

        def check(raw):
            return None if re.fullmatch(r'[\w.-]+', raw) else '영문·숫자·_ . - 만'

        name = self.ui.modal_input('지도 저장', [f'저장 위치: {self.bus.procs.map_dir_txt}/<이름>/map.yaml',
                                              f'빈칸 Enter = {default}'], '이름>', default, check)
        if name is None:
            return STAY
        if name in [n for n, _ in self.bus.procs.maps()] and not self.ui.confirm(f'{name} 이(가) 이미 있습니다. 덮어쓸까요?'):
            return STAY
        ok, msg = self.bus.procs.save_map(name)
        print(msg)
        return STAY


class SlamMenu(ServiceMenu):
    name = 'slam'

    def extra(self):
        return [SaveMapItem(self.app), RvizItem(self.app, 'slam'),
                TeleopItem(self.app, 't', '수동으로 이동하기 (텔레오퍼)'), AutoMapItem(self.app)]

    def hint(self):
        return [self.term.paint(' 맵핑: 1) 시작 → v) RViz로 보기 → t) 수동으로 돌기 또는 a) 자동 맵핑 → 3) 지도 저장', DIM),
                f' 탐색: {self.state.explore_status or "-"}' if self.state.st_control == 'SLAM_AUTO_MAPPING' else '']


class AutoMapItem(MenuItem):
    key = 'a'

    def label(self):
        return '자동 맵핑 중지' if self.state.st_control == 'SLAM_AUTO_MAPPING' else '자동 맵핑 시작 (스스로 탐색)'

    def run(self):
        if self.state.st_control == 'SLAM_AUTO_MAPPING':
            self.bus.cmd_automap(argparse.Namespace(value='stop'))
            return STAY
        body = ['explore_lite가 지도의 모르는 쪽 경계(frontier)로 Nav2 목표를 계속 보냅니다.',
                'SLAM · SLAM 중 Nav가 꺼져 있으면 켭니다 (Nav2 준비에 수십 초).',
                '주변을 비우세요. 끝나면 MANUAL로 돌아오고, 지도는 직접 저장 (3).']
        if self.ui.confirm('자동 맵핑을 시작할까요?', default=True, body=body):
            self.bus.cmd_automap(argparse.Namespace(value='start'))
        return STAY


# ---------------------------------------------------------------- Nav
class GoalItem(MenuItem):
    key = '4'

    def label(self):
        return '목표 보내기 (x y yaw°)'

    def run(self):
        raw = self.ui.modal_input('목표', ['map 좌표계 [m], 각도 [°]', 'RViz의 2D Goal Pose로 찍어도 됨'],
                                  '목표>', validate=_xyyaw)
        if raw is None:
            return STAY
        x, y, yaw = _parse_xyyaw(raw)
        self.bus.send_goal(x, y, yaw, wait=False)
        return STAY


class CancelGoalItem(MenuItem):
    key = '5'

    def label(self):
        return '목표 취소'

    def run(self):
        self.bus.cancel_goal()
        return STAY


class InitPoseItem(MenuItem):
    key = '3'

    def label(self):
        return '초기 위치 (x y yaw°)'

    def run(self):
        raw = self.ui.modal_input('초기 위치', ['amcl에 로봇이 지도 어디 있는지 알려줌',
                                            'RViz의 2D Pose Estimate로 찍어도 됨'],
                                  '위치>', '0 0 0', validate=_xyyaw)
        if raw is None:
            return STAY
        self.bus.set_initial_pose(*_parse_xyyaw(raw))
        return STAY


class MapStartItem(MenuItem):
    """Nav 시작 — 지도 고르는 화면으로."""

    key = '1'

    def label(self):
        return '시작 (지도 선택)'

    def visible(self):
        return not self.bus.procs.running('nav')

    def run(self):
        return MapPickMenu(self.app).show()


class PickItem(MenuItem):
    def __init__(self, app, key: str, name: str, path):
        super().__init__(app)
        self.key = key
        self.name = name
        self.path = path

    def label(self):
        return self.name

    def run(self):
        self.bus.start('nav', str(self.path))
        return BACK


class MapPickMenu(Menu):
    title = 'Nav — 지도 선택'

    def items(self):
        from .base import InfoItem
        maps = self.bus.procs.maps()
        if not maps:
            return [InfoItem(self.app, f'지도 없음 — SLAM으로 만들고 저장 ({self.bus.procs.map_dir_txt})')]
        return [PickItem(self.app, str(i), name, path) for i, (name, path) in enumerate(maps, 1)]

    def center(self):
        return ['[지도 목록] 최신 먼저', f' {self.bus.procs.map_dir_txt}', '',
                ' 고르면 nav2 bringup(map_server+amcl+주행)을 띄웁니다',
                ' 그다음 초기 위치(3)를 꼭 넣으세요']


class NavMenu(ServiceMenu):
    name = 'nav'

    def items(self):
        return [MapStartItem(self.app), StopItem(self.app, '1', self.name),
                RestartItem(self.app, '2', self.name), InitPoseItem(self.app),
                GoalItem(self.app), CancelGoalItem(self.app), RvizItem(self.app, 'nav'),
                LogItem(self.app, 'l', self.name)]

    def hint(self):
        info = self.bus.procs.info('nav')
        return ([f' 지도: {info}'] if info else []) + [
            self.term.paint(' 순서: 시작 → 초기 위치 → 목표', DIM),
            self.term.paint(' 보기: v) RViz로 보기', DIM)]


class ExploreMenu(ServiceMenu):
    name = 'explore'

    def extra(self):
        return [GoalItem(self.app), CancelGoalItem(self.app), RvizItem(self.app, 'slam')]

    def hint(self):
        warn = [] if self.bus.procs.running('slam') else [
            self.term.paint(' SLAM이 꺼져 있음 — SLAM 먼저 시작', YELLOW)]
        return warn + [self.term.paint(' SLAM 지도 위에서 목표로 움직이며 지도 넓히기', DIM),
                       self.term.paint(' 초기 위치 불필요 (SLAM이 map→odom을 냄)', DIM)]


class StopAllPage(Page):
    title = '전부 정지'

    def show(self) -> str:
        running = [SERVICES[n].label for n in SERVICES if self.bus.procs.running(n)]
        if not running:
            print('실행 중인 것 없음')
            return STAY
        body = ['실행 중: ' + ', '.join(running)]
        if self.bus.procs.running('slam'):
            body.append(self.term.paint('SLAM 지도는 저장하지 않으면 사라집니다', YELLOW))
        if self.ui.confirm('전부 끌까요?', body=body):
            self.bus.cmd_stop_all(None)
        return STAY
