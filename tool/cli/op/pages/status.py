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


class RebootMotorsItem(MenuItem):
    key = '2'

    def label(self):
        return '모터 재부팅 (에러 해제)'

    def run(self):
        body = [f'로봇: {self.state.robot_txt()}',
                '과열·과부하 등 모터 에러(E)로 토크가 꺼졌을 때 씁니다.',
                '두 바퀴를 재부팅합니다 — 잠깐 힘이 빠짐']
        if not self.ui.confirm('모터를 재부팅할까요?', body=body):
            return STAY
        self.bus.cmd_reboot_motors(None)
        return STAY


class StatusMenu(Menu):
    title = '상태'

    def items(self):
        return [PrintStatusItem(self.app), RebootMotorsItem(self.app)]

    def center(self):
        return self.bus.status_lines() + [
            '',
            ' 로봇 수신 없음 → 베이스(tb-only)가 Pi에서 떠 있는지 · PC면 ROS_STATIC_PEERS',
            ' 로봇 ✗ motor error → 2) 모터 재부팅 · 온도가 높으면(70℃↑) 식힌 뒤',
            ' odom 없음 → 베이스(2) 확인 · scan 없음 → 라이다 드라이버 확인',
            ' map 없음 → SLAM(3) 또는 Nav(4) 켜기',
            ' PC에서 안 보이면 → 같은 ROS_DOMAIN_ID · 같은 WiFi 확인']
