"""노드 목록 + 상태·파라미터·진단 live + 중지·재시작 (프로세스가 있는 로봇에서 실행 · PC는 SSH 경유)."""
from __future__ import annotations

import time

from ...defs import STAY
from ...terminal import BOLD, DIM, GREEN, RED, YELLOW
from ..base import GoMenu, InfoItem, Menu, MenuItem
from .core import CRITICAL, base


class KillNodeItem(MenuItem):
    key = '1'

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name

    def label(self):
        return '중지 (프로세스 kill)'

    def run(self):
        body = ['launch가 다시 띄우지 않음 — 서비스 재시작 전까지 멈춤']
        if base(self._name) in CRITICAL:
            body = [self.term.paint(f'제어 핵심 노드 — 끄면 {self._warn()}', YELLOW)] + body
        if not self.ui.confirm(f'{self._name} 노드를 중지할까요?', body=body, default=True):
            return STAY
        self.bus.send_velocity(0.0, 0.0)
        ok, msg = self.bus.procs.node_kill(self._name, force=True)
        print(msg)
        return STAY

    @staticmethod
    def _warn():
        return '바퀴 명령·odom이 멈춤'


class RestartNodeItem(MenuItem):
    key = '2'

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name

    def label(self):
        return '재시작 (kill 후 같은 명령으로 재실행)'

    def run(self):
        body = ['kill 전 실행 명령(argv) 그대로 다시 실행 · .run/nodes/에 추적']
        if base(self._name) in CRITICAL:
            body = [self.term.paint('제어 핵심 노드 — 재시작 중 잠깐 제어·odom이 끊김', YELLOW)] + body
        if not self.ui.confirm(f'{self._name} 노드를 재시작할까요?', body=body, default=True):
            return STAY
        self.bus.send_velocity(0.0, 0.0)
        ok, msg = self.bus.procs.node_restart(self._name, force=True)
        print(msg)
        return STAY


class NodeParamMenu(Menu):
    """노드 하나의 파라미터 현재값 + 진단 — 1초마다 자동 갱신 (읽기 전용)."""

    REFRESH_S = 1.0

    def __init__(self, app, name: str, ns: str):
        super().__init__(app)
        self.nodename = name
        self.nodens = ns
        disp = f'/{name}' if ns in ('/', '') else f'{ns.rstrip("/")}/{name}'
        self.title = f'파라미터 · {disp}'
        self._rows: list[tuple[str, object]] | None = None
        self._at = 0.0

    def refresh(self):
        self._rows = None
        self._at = 0.0

    def _fetch(self) -> list[tuple[str, object]]:
        now = time.monotonic()
        if self._rows is not None and now - self._at < self.REFRESH_S:
            return self._rows
        rows: list[tuple[str, object]] = []
        cli = self.bus.param_client(self.nodename, self.nodens)
        if cli is not None and cli.wait_for_services(timeout_sec=1.0):
            names = self.bus.param_list_names(cli) or []
            if names:
                descs = self.bus.param_describe(cli, names) or []
                vals = self.bus.param_get_vals(cli, names) or []
                for i, n in enumerate(names):
                    try:
                        v = (self.bus.param_display_value(descs[i].type, vals[i])
                             if i < len(descs) and i < len(vals) else None)
                    except Exception:
                        v = None
                    rows.append((n, v))
        self._rows = rows
        self._at = time.monotonic()
        return rows

    def items(self):
        rows = self._fetch()
        if not rows:
            return [InfoItem(self.app, '파라미터 없음 (서비스 무응답·권한 없음)')]
        return [InfoItem(self.app, f'{n} = {v!r}', key=str(i + 1))
                for i, (n, v) in enumerate(rows)]

    def center(self):
        full = f'/{self.nodename}' if self.nodens in ('/', '') else \
            f'{self.nodens.rstrip("/")}/{self.nodename}'
        lines = [f'[노드] {full}', ' 파라미터 현재값 — 1초마다 자동 갱신 (읽기 전용)', '']
        diags = [(n, lv, msg) for n, lv, msg in self.bus.diag_entries()
                 if self.nodename in n or full in n]
        for n, lv, msg in diags[:8]:
            mark = {0: 'OK', 1: 'WARN', 2: 'ERR', 3: 'STALE'}.get(lv, '?')
            lines.append(f' 진단 {self.bus.diag_short(n)}: {mark} {msg}')
        if not diags:
            lines.append(' 진단: 이 노드 항목 없음')
        return lines


class NodeActionMenu(Menu):
    """한 노드의 live 상태·진단 + 중지·재시작 + 파라미터 화면."""

    REFRESH_S = 1.0

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name
        self._lines: list[str] = []
        self._at = 0.0
        self._target = self.bus.param_target(name)

    @property
    def title(self):
        return f'노드 — {self._name}'

    def items(self):
        items: list[MenuItem] = [KillNodeItem(self.app, self._name),
                                 RestartNodeItem(self.app, self._name)]
        if self._target is not None:
            n, ns = self._target
            items.append(GoMenu(self.app, '3', '파라미터 · 진단 (live · 1초 갱신)',
                                NodeParamMenu(self.app, n, ns)))
        return items

    def _refresh_status(self):
        if self._lines and time.monotonic() - self._at < self.REFRESH_S:
            return
        self._lines = self.bus.node_status_lines(self._name)
        self._at = time.monotonic()

    def center(self):
        self._refresh_status()
        lines = list(self._lines)
        if self._target is not None:
            lines += ['', ' 3) 파라미터 · 진단 live 화면으로']
        if base(self._name) in CRITICAL:
            lines.append(self.term.paint(' 제어 핵심 노드 (state/cmd/opencr)', YELLOW))
        lines += ['', ' 중지: OS 프로세스 kill (launch respawn 없음 → 멈춤 유지)',
                  ' 재시작: kill → 같은 명령 재실행 (추적됨 · stop 때 정리)']
        return lines


class NodePickItem(MenuItem):
    shows_busy = False

    def __init__(self, app, key: str, row: tuple[str, bool, float]):
        super().__init__(app)
        self.key = key
        self._row = row

    def label(self):
        name, alive, age = self._row
        mark = self.term.paint('●', GREEN) if alive else self.term.paint('✗', RED + BOLD)
        text = f'{mark} {name}'
        if not alive:
            text += self.term.paint(f'  ({age:.0f}초 전 확인)', YELLOW)
        if base(name) in CRITICAL:
            text += self.term.paint(' *', BOLD)
        return text

    def run(self):
        return NodeActionMenu(self.app, self._row[0]).show()


class NodePickMenu(Menu):
    title = '노드 선택'
    REFRESH_S = 3.0  # 노드 목록 조회 주기 (매 리드로우마다 0.5초 폴링하지 않게)

    def __init__(self, app):
        super().__init__(app)
        self._rows: list[tuple[str, bool, float]] = []
        self._at = 0.0

    def _refresh(self):
        if self._rows and time.monotonic() - self._at < self.REFRESH_S:
            return
        self._rows = self.bus.node_rows()
        self._at = time.monotonic()

    def items(self):
        self._refresh()
        if not self._rows:
            return [InfoItem(self.app, '노드 없음 — 베이스가 꺼졌거나 ROS_DOMAIN_ID가 다름')]
        return [NodePickItem(self.app, str(i), r) for i, r in enumerate(self._rows, 1)]

    def center(self):
        return ['[중지·재시작할 노드 선택]', ' * = 제어 핵심 (확인창 뜸)', '',
                ' PC 등 다른 머신의 노드·컨테이너 내장 노드는 OS 프로세스가 없어 중지 불가']


class SelectNodeItem(MenuItem):
    key = '1'
    shows_busy = False

    def label(self):
        return '선택하여 상태·중지·재시작'

    def run(self):
        return NodePickMenu(self.app).show()


class PrintNodesItem(MenuItem):
    key = '2'

    def label(self):
        return '목록 출력'

    def run(self):
        names = self.bus.node_names()
        print('\n'.join(names) if names else '노드 없음')
        return STAY


class NodesMenu(Menu):
    title = '노드 (상태 · 파라미터 · 진단 · 중지 · 재시작)'
    REFRESH_S = 3.0

    def __init__(self, app):
        super().__init__(app)
        self._rows: list[tuple[str, bool, float]] = []
        self._at = 0.0

    def _refresh(self):
        if self._rows and time.monotonic() - self._at < self.REFRESH_S:
            return
        self._rows = self.bus.node_rows()
        self._at = time.monotonic()

    def items(self):
        self._refresh()
        return [SelectNodeItem(self.app), PrintNodesItem(self.app)]

    def center(self):
        self._refresh()
        if not self._rows:
            return ['노드 없음 — 베이스/SLAM/Nav가 꺼졌거나 ROS_DOMAIN_ID가 다름']
        n_alive = sum(1 for _, alive, _ in self._rows if alive)
        lines = [f'[노드 {n_alive}개 활성]']
        for name, alive, age in self._rows:
            if alive:
                lines.append(f' {name}')
            else:
                lines.append(f' {self.term.paint(name + f" — 사라짐 ({age:.0f}초 전 확인)", YELLOW)}')
        bad = [(n, lv, m) for n, lv, m in self.bus.diag_entries() if lv >= 1]
        lines.append('')
        if bad:
            lines.append(self.term.paint(f' 진단 주의 {len(bad)}개', YELLOW))
            for n, lv, m in bad[:5]:
                lines.append(f'  - {self.bus.diag_short(n)}: {m}')
        elif self.state.diagnostics:
            lines.append(self.term.paint(f' 진단 OK ({len(self.state.diagnostics)}개)', GREEN))
        else:
            lines.append(' 진단: -')
        if any(base(name) == 'controller_server' for name, _, _ in self._rows):
            lines += self.bus.node_feature_lines('controller_server', summary=True)
        return lines


from .feature import NodeFeature
from .controller_server import ControllerServerFeature

FEATURES: tuple[NodeFeature, ...] = (ControllerServerFeature(),)


def feature_for(name: str) -> NodeFeature | None:
    """노드 이름(네임스페이스 포함 가능) → 추가 상태 feature."""
    short = name.rstrip('/').split('/')[-1]
    for feature in FEATURES:
        if feature.node == short:
            return feature
    return None


def wire_all(tool) -> None:
    """RobotTool 기동 시 각 feature의 ROS 구독을 만든다."""
    for feature in FEATURES:
        feature.wire(tool)
