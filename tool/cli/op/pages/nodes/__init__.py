"""노드 목록 + 상태·파라미터·진단 live + 중지·재시작 (프로세스가 있는 로봇에서 실행 · PC는 SSH 경유)."""
from __future__ import annotations

import textwrap
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


class ParamItem(MenuItem):
    """파라미터 한 줄 — Enter로 값 변경 (형식은 descriptor.type에 맞춤)."""

    shows_busy = False

    # rcl_interfaces/msg/ParameterType 값 (rclpy 없이도 표시/파싱)
    P_BOOL, P_INT, P_DOUBLE, P_STRING = 1, 2, 3, 4
    P_BYTE_ARRAY, P_BOOL_ARRAY, P_INT_ARRAY, P_DOUBLE_ARRAY, P_STRING_ARRAY = 5, 6, 7, 8, 9

    def __init__(self, app, menu, name: str, value, desc, cli, key: str = ''):
        super().__init__(app)
        self.key = key
        self.menu = menu
        self.name = name
        self.value = value
        self.desc = desc
        self.cli = cli

    @classmethod
    def type_name(cls, t) -> str:
        return {cls.P_BOOL: 'bool', cls.P_INT: 'int', cls.P_DOUBLE: 'double',
                cls.P_STRING: 'string', cls.P_BYTE_ARRAY: 'byte[]',
                cls.P_BOOL_ARRAY: 'bool[]', cls.P_INT_ARRAY: 'int[]',
                cls.P_DOUBLE_ARRAY: 'double[]', cls.P_STRING_ARRAY: 'string[]'}.get(
                    t, f'type {t}')

    @classmethod
    def fmt(cls, t, v) -> str:
        if t == cls.P_BOOL:
            return 'true' if v else 'false'
        if t in (cls.P_BYTE_ARRAY, cls.P_BOOL_ARRAY, cls.P_INT_ARRAY,
                 cls.P_DOUBLE_ARRAY, cls.P_STRING_ARRAY):
            return ', '.join(str(x) for x in (v or []))
        return '' if v is None else str(v)

    @staticmethod
    def _split(raw: str) -> list[str]:
        s = raw.strip()
        if s.startswith('[') and s.endswith(']'):
            s = s[1:-1]
        if not s.strip():
            return []
        return [p.strip() for p in (s.split(',') if ',' in s else s.split())]

    @classmethod
    def parse(cls, t, raw: str):
        if t == cls.P_STRING:
            return raw
        s = raw.strip()
        if t == cls.P_BOOL:
            low = s.lower()
            if low in ('1', 'true', 'on', 'yes', 'y'):
                return True
            if low in ('0', 'false', 'off', 'no', 'n'):
                return False
            raise ValueError('bool: true/false/1/0/on/off')
        if t == cls.P_INT:
            return int(s, 0)
        if t == cls.P_DOUBLE:
            return float(s)
        parts = cls._split(s)
        if t == cls.P_BYTE_ARRAY:
            vals = [int(p, 0) for p in parts]
            if any(v < 0 or v > 255 for v in vals):
                raise ValueError('byte는 0~255')
            return vals
        if t == cls.P_BOOL_ARRAY:
            vals = []
            for p in parts:
                low = p.lower()
                if low in ('1', 'true', 'on', 'yes', 'y'):
                    vals.append(True)
                elif low in ('0', 'false', 'off', 'no', 'n'):
                    vals.append(False)
                else:
                    raise ValueError(f'bool 아님: {p!r}')
            return vals
        if t == cls.P_INT_ARRAY:
            return [int(p, 0) for p in parts]
        if t == cls.P_DOUBLE_ARRAY:
            return [float(p) for p in parts]
        if t == cls.P_STRING_ARRAY:
            return [p.strip() for p in s.split(',')] if s else []
        raise ValueError(f'지원하지 않는 형식: {t}')

    def label(self):
        ro = ' [읽기전용]' if getattr(self.desc, 'read_only', False) else ''
        return f'{self.name} = {self.value!r}{ro}'

    def run(self):
        ptype = getattr(self.desc, 'type', None)
        if ptype is None:
            print('형식 정보 없음 — 값을 바꿀 수 없습니다')
            return STAY
        if getattr(self.desc, 'read_only', False):
            print(f'{self.name}: 읽기 전용 파라미터')
            return STAY
        raw = self.ui.modal_input(
            f'파라미터 · {self.name}',
            [f'형식: {self.type_name(ptype)}', f'현재: {self.fmt(ptype, self.value)}',
             'Enter 적용 · Esc 취소'],
            '값>', default=self.fmt(ptype, self.value),
            validate=lambda t: self._validate(ptype, t))
        if raw is None:
            return STAY
        try:
            value = self.parse(ptype, raw)
        except (ValueError, TypeError) as exc:
            print(f'값 변환 실패: {exc}')
            return STAY
        ok, msg = self.bus.param_set(self.cli, self.name, ptype, value)
        print(('설정: ' if ok else 'FAIL: ') + f'{self.name} = {value!r}'
              + ('' if ok else f' — {msg}'))
        if ok:
            self.menu.refresh()
            self._runtime_note()
        return STAY

    @classmethod
    def _validate(cls, ptype, raw):
        try:
            cls.parse(ptype, raw)
        except (ValueError, TypeError) as exc:
            return str(exc)
        return None

    def _runtime_note(self):
        n = self.name
        if n.endswith('FollowPath.model_path'):
            print('model_path는 controller_server가 configure될 때만 읽습니다 — nav restart 후 적용')
        elif n.endswith('FollowPath.enable'):
            print('enable 파라미터는 configure 때만 읽습니다 — 실제 토글은 ab rl_nav_controller on/off')
        elif n.endswith('FollowPath.lidar_inflation'):
            print('다음 제어 주기부터 적용됩니다 (재시작 불필요)')


class NodeParamMenu(Menu):
    """노드 하나의 파라미터 현재값 + 진단 — Enter로 값 변경 · 1초마다 자동 갱신."""

    REFRESH_S = 1.0

    def __init__(self, app, name: str, ns: str):
        super().__init__(app)
        self.nodename = name
        self.nodens = ns
        disp = f'/{name}' if ns in ('/', '') else f'{ns.rstrip("/")}/{name}'
        self.title = f'파라미터 · {disp}'
        self._rows: list[tuple[str, object, object]] | None = None
        self._cli = None
        self._at = 0.0

    def refresh(self):
        self._rows = None
        self._at = 0.0

    def _fetch(self) -> list[tuple[str, object, object]]:
        now = time.monotonic()
        if self._rows is not None and now - self._at < self.REFRESH_S:
            return self._rows
        rows: list[tuple[str, object, object]] = []
        cli = self.bus.param_client(self.nodename, self.nodens)
        self._cli = cli
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
                    rows.append((n, v, descs[i] if i < len(descs) else None))
        self._rows = rows
        self._at = time.monotonic()
        return rows

    def items(self):
        rows = self._fetch()
        if not rows:
            return [InfoItem(self.app, '파라미터 없음 (서비스 무응답·권한 없음)')]
        return [ParamItem(self.app, self, n, v, d, self._cli, str(i + 1))
                for i, (n, v, d) in enumerate(rows)]

    def center(self):
        full = f'/{self.nodename}' if self.nodens in ('/', '') else \
            f'{self.nodens.rstrip("/")}/{self.nodename}'
        rows = self._fetch()
        lines = [f'[노드] {full}', ' ↑↓ 선택 → 오른쪽 상세 · Enter 수정 · 1초 갱신', '']
        if rows:
            idx = max(0, min(self.ui.window.menu_bar.idx, len(rows) - 1))   # 포커스된 항목
            n, v, d = rows[idx]
            ptype = getattr(d, 'type', None)
            lines.append('[선택한 파라미터]')
            lines.append(f' {n}')
            if ptype is not None:
                lines.append(f' 형식: {ParamItem.type_name(ptype)}')
                lines.append(f' 현재: {ParamItem.fmt(ptype, v)}')
            else:
                lines.append(f' 현재: {v!r}')
            lines.append(f' 읽기전용: {"yes" if getattr(d, "read_only", False) else "no"}')
            desc = (getattr(d, 'description', '') or '').strip()
            lines.append(' 설명:')
            if desc:
                lines += [f'  {x}' for x in textwrap.wrap(desc, 46)]
            else:
                lines.append('  (설명 없음)')
            extra = (getattr(d, 'additional_constraints', '') or '').strip()
            if extra:
                lines.append(' 제약:')
                lines += [f'  {x}' for x in textwrap.wrap(extra, 46)]
            ranges = list(getattr(d, 'integer_range', []) or []) + \
                list(getattr(d, 'floating_point_range', []) or [])
            if ranges:
                r = ranges[0]
                lines.append(f' 범위: {r.from_value} ~ {r.to_value} (step {r.step})')
            lines.append('')
        diags = [(n, lv, msg) for n, lv, msg in self.bus.diag_entries()
                 if self.nodename in n or full in n]
        if diags:
            lines.append(f' 진단 {len(diags)}개')
            for n, lv, msg in diags[:8]:
                mark = {0: 'OK', 1: 'WARN', 2: 'ERR', 3: 'STALE'}.get(lv, '?')
                lines.append(f'  {self.bus.diag_short(n)}: {mark} {msg}')
        else:
            bad = sum(1 for _, lv, _ in self.bus.diag_entries() if lv >= 1)
            lines.append(' 진단: 이 노드 항목 없음' + (f' · 전체 경고/에러 {bad}개' if bad else ''))
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
            items.append(GoMenu(self.app, '3', '파라미터 설정 · 진단 (live · 1초 갱신)',
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
            lines += ['', ' 3) 파라미터 설정 · 진단 live 화면으로']
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
