"""상태 화면 — 2초마다 갱신되는 요약 + 한 번 출력."""
from __future__ import annotations

from ..defs import STAY
from .base import Menu, MenuItem


class PrintStatusItem(MenuItem):
    key = '1'

    def label(self):
        return '상태 출력 (복사용)'

    def run(self):
        print('\n'.join(self.bus.status_lines()))
        return STAY


class StatusMenu(Menu):
    title = '상태'

    def items(self):
        return [PrintStatusItem(self.app)]

    def center(self):
        return self.bus.status_lines() + [
            '',
            ' odom 없음 → 베이스(2) 확인 · scan 없음 → 라이다 드라이버 확인',
            ' map 없음 → SLAM(3) 또는 Nav(4) 켜기',
            ' PC에서 안 보이면 → 같은 ROS_DOMAIN_ID · 같은 WiFi 확인']
