"""상태 화면 — 2초마다 갱신되는 요약 + 한 번 출력."""
from __future__ import annotations

import argparse

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


class ToggleItem(MenuItem):
    """모드·RL·비상정지 전환 — state_manager에 /cmd_manual_* 요청."""

    def __init__(self, app, key: str, what: str):
        super().__init__(app)
        self.key = key
        self.what = what

    def label(self):
        st = self.state
        return {'mode': f'모드 전환 (지금 {st.st_mode or "?"})',
                'rl': f'RL 보정 {"끄기" if st.st_rl else "켜기"}',
                'estop': f'비상정지 {"해제" if st.st_estop else "ON"}'}[self.what]

    def run(self):
        st = self.state
        if self.what == 'mode':
            value = 'AUTO' if st.st_mode == 'MANUAL' else 'MANUAL'
            self.bus.cmd_mode(argparse.Namespace(value=value))
        elif self.what == 'rl':
            body = ['켜면 바퀴 명령이 PC의 rl-only(rl_corrector)를 거칩니다.',
                    'PC에서 ros2 launch tb3_bringup rl-only.launch.py 가 없으면 바퀴가 멈춥니다.']
            if st.st_rl or self.ui.confirm('RL 보정을 켤까요?', body=body):
                self.bus.cmd_rl(argparse.Namespace(value='off' if st.st_rl else 'on'))
        elif st.st_estop:
            if self.ui.confirm('비상정지를 해제할까요?', body=['해제하면 다시 명령대로 움직입니다.']):
                self.bus.cmd_estop(argparse.Namespace(value='off'))
        else:
            self.bus.cmd_estop(argparse.Namespace(value='on'))  # ON은 확인 없이 바로
        return STAY


class StatusMenu(Menu):
    title = '상태'

    def items(self):
        return [PrintStatusItem(self.app), RebootMotorsItem(self.app), ToggleItem(self.app, '3', 'mode'),
                ToggleItem(self.app, '4', 'rl'), ToggleItem(self.app, '5', 'estop')]

    def center(self):
        return self.bus.status_lines() + [
            '',
            ' 로봇 수신 없음 → 베이스(tb-only)가 Pi에서 떠 있는지 · PC면 ROS_STATIC_PEERS',
            ' 로봇 ✗ motor error → 2) 모터 재부팅 · 온도가 높으면(70℃↑) 식힌 뒤',
            ' odom 없음 → 베이스(2) 확인 · scan 없음 → 라이다 드라이버 확인',
            ' map 없음 → SLAM(3) 또는 Nav(4) 켜기',
            ' PC에서 안 보이면 → 같은 ROS_DOMAIN_ID · 같은 WiFi 확인']
