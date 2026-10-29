"""노드 목록 + 선택 중지·재시작 (프로세스가 있는 로봇에서 실행 · PC는 SSH 경유)."""
from __future__ import annotations

from ..defs import STAY
from ..nodes import CRITICAL
from ..terminal import YELLOW
from .base import InfoItem, Menu, MenuItem


class KillNodeItem(MenuItem):
    key = '1'

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name

    def label(self):
        return '중지 (프로세스 kill)'

    def run(self):
        body = ['launch가 다시 띄우지 않음 — 서비스 재시작 전까지 멈춤']
        if self._name in CRITICAL:
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
        if self._name in CRITICAL:
            body = [self.term.paint('제어 핵심 노드 — 재시작 중 잠깐 제어·odom이 끊김', YELLOW)] + body
        if not self.ui.confirm(f'{self._name} 노드를 재시작할까요?', body=body, default=True):
            return STAY
        self.bus.send_velocity(0.0, 0.0)
        ok, msg = self.bus.procs.node_restart(self._name, force=True)
        print(msg)
        return STAY


class NodeActionMenu(Menu):
    """한 노드의 중지·재시작."""

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name

    @property
    def title(self):
        return f'노드 — {self._name}'

    def items(self):
        return [KillNodeItem(self.app, self._name), RestartNodeItem(self.app, self._name)]

    def center(self):
        lines = [f'[노드 {self._name}]']
        if self._name in CRITICAL:
            lines.append(self.term.paint(' 제어 핵심 노드 (state/cmd/opencr)', YELLOW))
        lines += ['', ' 중지: OS 프로세스 kill (launch respawn 없음 → 멈춤 유지)',
                  ' 재시작: kill → 같은 명령 재실행 (추적됨 · stop-all 때 정리)']
        return lines


class NodePickItem(MenuItem):
    shows_busy = False

    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self._name = name

    def label(self):
        mark = ' *' if self._name in CRITICAL else ''
        return f'{self._name}{mark}'

    def run(self):
        return NodeActionMenu(self.app, self._name).show()


class NodePickMenu(Menu):
    title = '노드 선택'

    def items(self):
        names = self.bus.node_names()
        if not names:
            return [InfoItem(self.app, '노드 없음 — 베이스가 꺼졌거나 ROS_DOMAIN_ID가 다름')]
        return [NodePickItem(self.app, str(i), n.lstrip('/').split('/')[-1])
                for i, n in enumerate(names, 1)]

    def center(self):
        return ['[중지·재시작할 노드 선택]', ' * = 제어 핵심 (확인창 뜸)', '',
                ' PC 등 다른 머신의 노드·컨테이너 내장 노드는 OS 프로세스가 없어 중지 불가']


class SelectNodeItem(MenuItem):
    key = '1'
    shows_busy = False

    def label(self):
        return '선택하여 중지·재시작'

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
    title = '노드'

    def items(self):
        return [SelectNodeItem(self.app), PrintNodesItem(self.app)]

    def center(self):
        names = self.bus.node_names()
        return [f'[노드 {len(names)}개]'] + [f' {n}' for n in names] if names else [
            '노드 없음 — 베이스/SLAM/Nav가 꺼졌거나 ROS_DOMAIN_ID가 다름']
