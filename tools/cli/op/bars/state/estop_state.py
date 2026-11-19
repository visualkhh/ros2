"""EstopState — 비상정지 칸 (E-STOP ON / OFF). Tab으로 골라 Enter = 켜기/해제 (다이얼로그 · y)."""
from __future__ import annotations

from ...defs import STAY
from ...terminal import BOLD, GREEN, RED, YELLOW
from .state import State


class EstopState(State):
    focusable = True

    def activate(self):
        self.ui.bus.toggle_estop()   # 상태 메뉴의 '비상정지 ON/해제'와 같은 경로
        return STAY

    def text(self) -> str:
        on = self.state.st_estop
        if on is None:
            return 'E-STOP ' + self.term.paint('-', YELLOW)
        return 'E-STOP ' + (self.term.paint('ON', RED + BOLD) if on else self.term.paint('OFF', GREEN))
