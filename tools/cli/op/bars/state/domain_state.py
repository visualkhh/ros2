"""DomainState — state_manager의 /state_*: 제어 · 운행(지도) · 비상정지 (비상정지면 빨강, 상태 없으면 노랑)."""
from __future__ import annotations

from ...terminal import BOLD, CYAN, RED, YELLOW
from .state import State


class DomainState(State):
    def text(self) -> str:
        st = self.state
        color = RED + BOLD if st.st_estop else (YELLOW if st.st_control is None else CYAN)
        return self.term.paint(st.domain_txt(), color)
