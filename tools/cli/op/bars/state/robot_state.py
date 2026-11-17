"""RobotState — 모터(OpenCR) 상태: opencr_driver의 /diagnostics — 연결·모터 에러·온도·전류."""
from __future__ import annotations

from ...terminal import BOLD, GREEN, RED, YELLOW
from .state import State


class RobotState(State):
    def text(self) -> str:
        st = self.state
        level, summary = st.robot_parts()
        color = {0: GREEN, 1: YELLOW, 2: RED + BOLD}.get(level, YELLOW)
        mark = {0: '●', 1: '▲', 2: '✗'}.get(level, '?')
        hint = ' → 상태 메뉴 2) 모터 재부팅' if level == 2 and 'motor error' in st.robot_msg else ''
        summary = summary.split(' · 배터리 ', 1)[0]   # 배터리는 PowerState 칸에서
        if summary.startswith('모터 '):
            summary = summary[3:]
        return self.term.paint(f'모터 {mark} {summary}{hint}', color)
