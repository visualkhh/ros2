"""EstopState — 비상정지 칸 (E-STOP ON / OFF)."""
from __future__ import annotations

from ...terminal import BOLD, GREEN, RED, YELLOW
from .state import State


class EstopState(State):
    def text(self) -> str:
        on = self.state.st_estop
        if on is None:
            return 'E-STOP ' + self.term.paint('-', YELLOW)
        return 'E-STOP ' + (self.term.paint('ON', RED + BOLD) if on else self.term.paint('OFF', GREEN))
