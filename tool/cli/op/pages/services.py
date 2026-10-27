"""프로세스 메뉴 — 베이스 · SLAM 맵핑 · Nav · SLAM 중 Nav (start/stop/재시작/로그 + 전용 항목)."""
from __future__ import annotations

import math
import re

from ..defs import BACK, STAY
from ..procs import MAP_DIR, SERVICES
from ..terminal import DIM, GREEN, YELLOW
from .base import Menu, MenuItem, Page


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
        info = self.bus.procs.info(self.name) or None
        self.bus.stop(self.name)
        self.bus.start(self.name, info)
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


# ---------------------------------------------------------------- SLAM
class SaveMapItem(MenuItem):
    key = '3'

    def label(self):
        return '지도 저장'

    def run(self):
        default = self.bus.default_map_name()

        def check(raw):
            return None if re.fullmatch(r'[\w.-]+', raw) else '영문·숫자·_ . - 만'

        name = self.ui.modal_input('지도 저장', [f'저장 위치: {MAP_DIR}/<이름>/map.yaml',
                                              f'빈칸 Enter = {default}'], '이름>', default, check)
        if name is None:
            return STAY
        if (MAP_DIR / name / 'map.yaml').exists() and not self.ui.confirm(f'{name} 이(가) 이미 있습니다. 덮어쓸까요?'):
            return STAY
        ok, msg = self.bus.procs.save_map(name)
        print(msg)
        return STAY


class SlamMenu(ServiceMenu):
    name = 'slam'

    def extra(self):
        return [SaveMapItem(self.app)]

    def hint(self):
        return [self.term.paint(' 맵핑: 시작 → 텔레오퍼(메인 5)로 천천히 돌기 → 지도 저장', DIM),
                self.term.paint(' 보기: PC에서 tool/run-rviz slam', DIM)]


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
            return [InfoItem(self.app, f'지도 없음 — SLAM으로 만들고 저장 ({MAP_DIR})')]
        return [PickItem(self.app, str(i), name, path) for i, (name, path) in enumerate(maps, 1)]

    def center(self):
        return ['[지도 목록] 최신 먼저', f' {MAP_DIR}', '',
                ' 고르면 nav2 bringup(map_server+amcl+주행)을 띄웁니다',
                ' 그다음 초기 위치(3)를 꼭 넣으세요']


class NavMenu(ServiceMenu):
    name = 'nav'

    def items(self):
        return [MapStartItem(self.app), StopItem(self.app, '1', self.name),
                RestartItem(self.app, '2', self.name), InitPoseItem(self.app),
                GoalItem(self.app), CancelGoalItem(self.app), LogItem(self.app, 'l', self.name)]

    def hint(self):
        info = self.bus.procs.info('nav')
        return ([f' 지도: {info}'] if info else []) + [
            self.term.paint(' 순서: 시작 → 초기 위치 → 목표', DIM),
            self.term.paint(' 보기: PC에서 tool/run-rviz nav', DIM)]


class ExploreMenu(ServiceMenu):
    name = 'explore'

    def extra(self):
        return [GoalItem(self.app), CancelGoalItem(self.app)]

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
