"""메인 메뉴."""
from __future__ import annotations

from ..defs import MAIN
from .base import GoMenu, Menu

ENTRIES = [
    ('1', '상태'),
    ('2', '베이스 (바퀴·odom·라이다)'),
    ('3', 'SLAM 맵핑 (지도 만들기)'),
    ('4', 'Nav (저장된 지도로 주행)'),
    ('5', '수동 주행 / 텔레오퍼'),
    ('6', '지도 파일'),
    ('7', 'SLAM 중 Nav (목표로 탐색)'),
    ('8', '노드 (목록·중지·재시작)'),
    ('9', '전부 정지·재시작'),
    ('0', '비프음 (삑·톤·타이핑)'),
]


class MainMenu(Menu):
    title = '메인 메뉴'
    root = True

    def items(self):
        return [GoMenu(self.app, key, label, self.app.screen(key)) for key, label in ENTRIES]

    def center(self):
        return self.bus.status_lines() + ['', self.term.paint(
            ' 화면 보기: 3) SLAM · 4) Nav · 7) 메뉴의 v) RViz로 보기 (또는 ./run-rviz slam|nav)', '\033[2m')]

    def show(self) -> int:
        self.app.poll(0.6)
        while super().show() == MAIN:  # 하위 화면의 m = 여기로 돌아옴
            pass
        self.term.clear()
        print('robot_tool 종료 (띄운 SLAM·Nav는 계속 돕니다 — 끄려면 ./run-cli stop-all)')
        return 0
