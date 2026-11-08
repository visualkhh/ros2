"""RobotState — 로봇(OpenCR) 상태: opencr_driver의 /diagnostics — 연결·모터 에러·온도·전류."""
from __future__ import annotations

from ..terminal import BOLD, GREEN, RED, YELLOW
from .state import State


class RobotState(State):
    def text(self) -> str:
        st = self.state
        level, summary = st.robot_parts()
        color = {0: GREEN, 1: YELLOW, 2: RED + BOLD}.get(level, YELLOW)
        mark = {0: '●', 1: '▲', 2: '✗'}.get(level, '?')
        hint = ' → 상태 메뉴 2) 모터 재부팅' if level == 2 and 'motor error' in st.robot_msg else ''
        return self.term.paint(f'로봇 {mark} {summary}{hint}', color)
