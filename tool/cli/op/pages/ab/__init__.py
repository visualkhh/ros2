"""A/B 테스트 — Nav2 FollowPath RL 정책 ON/OFF + rosbag 기록.

메뉴 `b` → rl_nav_controller 화면.
"""
from __future__ import annotations

import sys
import time

from ...defs import STAY
from ..base import GoMenu, Menu, MenuItem
from .core import AB_NODES


class AbStatusItem(MenuItem):
    key = '1'

    def label(self):
        return '상태 (RL/MPPI · 모델 · 스캔 · 녹화)'

    def run(self):
        print('\n'.join(self.bus.ab.status_lines()))
        return STAY


class AbToggleItem(MenuItem):
    key = '2'

    def label(self):
        st = self.bus.ab.status()
        if not st['fresh']:
            return '정책 ON/OFF — controller_server 상태 없음'
        return f'정책 {"끄기" if st["enabled"] else "켜기"} (현재 {"RL" if st["enabled"] else st["inner"] + " 바이패스"})'

    def run(self):
        st = self.bus.ab.status()
        if not st['fresh']:
            print('FAIL: controller_server 상태 없음 — Nav2가 도는지 확인', file=sys.stderr)
            return STAY
        on = not st['enabled']
        if not self.ui.confirm(
                f'FollowPath RL을 {"ON" if on else "OFF"}로 바꿀까요?', default=True,
                body=['OFF = inner controller(MPPI/DWB) — 비교 기준', 'A/B는 주행과 주행 사이에 바꾸세요.']):
            return STAY
        ok, msg = self.bus.ab.toggle(on)
        print(('ok: ' if ok else 'FAIL: ') + msg)
        return STAY


class AbRecordItem(MenuItem):
    key = '3'

    def label(self):
        return '녹화 시작 (rosbag2 · 백그라운드)'

    def run(self):
        if self.bus.ab.current():
            print('FAIL: 이미 녹화 중 — 4) 녹화 종료를 먼저 누르세요', file=sys.stderr)
            return STAY
        label = self.ui.modal_input(
            'A/B 녹화', ['라벨 (예: A, B, rl, mppi)', '저장: tool/bags/ab/<시각>_<라벨>'], '라벨>', default='A')
        if label is None:
            return STAY
        try:
            rec = self.bus.ab.start(label)
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return STAY
        print(f'녹화 시작: {rec["bag"]}')
        print('주행을 마친 뒤 4) 녹화 종료를 누르세요.')
        return STAY


class AbStopItem(MenuItem):
    key = '4'

    def label(self):
        return '녹화 종료 (rosbag2 마무리)'

    def run(self):
        rec = self.bus.ab.stop()
        if rec is None:
            print('녹화 중 아님')
            return STAY
        dur = max(0.0, float(rec.get('stopped', time.time())) - float(rec.get('started', time.time())))
        print(f'녹화 종료: {rec["bag"]} ({dur:.1f}s)')
        return STAY


class AbGuideItem(MenuItem):
    key = '6'

    def label(self):
        return '자동 A/B (command.json → Nav2 주행)'

    def run(self):
        label = self.ui.modal_input(
            '자동 A/B', ['command.json의 route대로 A(RL) → B(inner)를 주행합니다.',
                         'Nav2 · NAV_PLAN · E-STOP OFF 필요', '라벨을 입력하세요'],
            '라벨>', default=time.strftime('%Y%m%d_%H%M%S'))
        if label is None:
            return STAY
        rc = self.bus.ab.guide(label)
        if rc != 0:
            print(f'FAIL: 자동 A/B 실패 (rc={rc})', file=sys.stderr)
        return STAY


class AbHistoryItem(MenuItem):
    key = '5'

    def label(self):
        return f'녹화 목록 ({len(self.bus.ab.history())}개)'

    def run(self):
        hist = self.bus.ab.history()
        if not hist:
            print('녹화 없음')
        for h in hist:
            print(f'{h.get("label", "?"):<12} {h["bag"]}')
        return STAY


class RlNavControllerAbMenu(Menu):
    title = 'A/B 테스트 · rl_nav_controller'
    REFRESH_S = 1.0

    def __init__(self, app):
        super().__init__(app)
        self._lines: list[str] = []
        self._at = 0.0

    def items(self):
        return [AbStatusItem(self.app), AbToggleItem(self.app), AbRecordItem(self.app),
                AbStopItem(self.app), AbGuideItem(self.app), AbHistoryItem(self.app)]

    def center(self):
        now = time.monotonic()
        if not self._lines or now - self._at >= self.REFRESH_S:
            self._lines = self.bus.ab.status_lines()
            self._at = now
        return self._lines + ['', ' 같은 출발점·같은 goal로 RL ON / OFF를 번갈아 여러 번']


def ab_node_pages(app) -> list[tuple[str, str, Menu]]:
    pages = {'rl_nav_controller': RlNavControllerAbMenu}
    return [(name, desc, pages[name](app)) for name, desc in AB_NODES]


class AbMenu(Menu):
    title = 'A/B 테스트 (노드별)'

    def items(self):
        return [GoMenu(self.app, str(i + 1), f'{name} — {desc}', page)
                for i, (name, desc, page) in enumerate(ab_node_pages(self.app))]

    def center(self):
        lines = ['[A/B 대상 노드]', ' rl_nav_controller — Nav2 FollowPath RL/inner']
        lines += ['', ' 녹화: tool/bags/ab/<시각>_<라벨> (rosbag2)',
                  ' 자동: rl_nav_controller/command.json의 route']
        return lines
