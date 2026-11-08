"""ProcState — launch 프로세스 한 칸 (켜짐 ON · 꺼짐 off). 서비스(베이스·SLAM·Nav…)마다 하나."""
from __future__ import annotations

from ..terminal import BOLD, DIM, GREEN
from .state import State


class ProcState(State):
    def __init__(self, ui, name: str, label: str):
        super().__init__(ui)
        self.name, self.label = name, label

    def text(self) -> str:
        on = self.ui.procs.running(self.name)
        return self.term.paint(f'{self.label} {"ON" if on else "off"}', GREEN + BOLD if on else DIM)
