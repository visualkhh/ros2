"""상태 화면 — 2초마다 갱신되는 요약 + 한 번 출력."""
from __future__ import annotations

import argparse

from ..defs import BACK, STAY
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
        return {'mode': f'제어 상태 바꾸기 (지금 {st.st_control or "?"})',
                'rl': f'경로 바꾸기 (지금 {"RL 경유" if st.st_rl else "직결"} → {"직결" if st.st_rl else "RL 경유"})',
                'estop': f'비상정지 {"해제" if st.st_estop else "ON"}'}[self.what]

    def run(self):
        st = self.state
        if self.what == 'mode':
            return ModeMenu(self.app).show()
        elif self.what == 'rl':
            body = ['RL 경유: cmd_manager → PC rl_corrector(rl-only) → cmd_manager → opencr_driver.',
                    'PC에서 ros2 launch tb3_bringup rl-only.launch.py 가 없으면 바퀴가 멈춥니다.']
            if st.st_rl or self.ui.confirm('경로를 RL 경유로 바꿀까요?', body=body):
                self.bus.cmd_route(argparse.Namespace(value='direct' if st.st_rl else 'rl'))
        elif st.st_estop:
            if self.ui.confirm('비상정지를 해제할까요?', body=['해제하면 다시 명령대로 움직입니다.']):
                self.bus.cmd_estop(argparse.Namespace(value='off'))
        else:
            self.bus.cmd_estop(argparse.Namespace(value='on'))  # ON은 확인 없이 바로
        return STAY


class ModeItem(MenuItem):
    def __init__(self, app, key: str, mode: str, desc: str):
        super().__init__(app)
        self.key, self.mode, self.desc = key, mode, desc

    def label(self):
        return f'{self.mode:<13} {self.desc}' + ('  ← 지금' if self.state.st_control == self.mode else '')

    def run(self):
        if self.mode == 'SLAM_AUTO_MAPPING':
            self.bus.cmd_automap(argparse.Namespace(value='start'))
        else:
            self.bus.cmd_mode(argparse.Namespace(value=self.mode))
        return BACK


class ModeMenu(Menu):
    title = '제어 상태'

    def items(self):
        return [ModeItem(self.app, '1', 'MANUAL', '텔레오퍼·drive 명령만 바퀴로'),
                ModeItem(self.app, '2', 'NAV_PLAN', 'Nav2 목표 주행 (goal · RViz Nav2 Goal)'),
                ModeItem(self.app, '3', 'SLAM_AUTO_MAPPING', '자동 맵핑 — SLAM·SLAM 중 Nav를 켜고 스스로 탐색')]

    def center(self):
        return ['[제어 상태] 바꾸는 판단은 cmd_manager, 알리는 건 state_manager (/state_control)', '',
                ' MANUAL에서만 수동 명령, NAV_PLAN·SLAM_AUTO_MAPPING에서만 Nav2 명령이 바퀴로 갑니다.',
                ' RViz Nav2 Goal을 찍으면 자동으로 NAV_PLAN, 자동 맵핑이 끝나면 MANUAL로 돌아옵니다.']


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
