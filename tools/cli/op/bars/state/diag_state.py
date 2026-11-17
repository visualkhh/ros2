"""DiagState — 진단 요약 칸 (OK 수 · 주의 수)."""
from __future__ import annotations

from ...terminal import BOLD, DIM, GREEN, RED, YELLOW
from .state import State


class DiagState(State):
    def text(self) -> str:
        diags = self.state.diagnostics
        if not diags:
            return self.term.paint('진단 -', DIM)
        bad = [lv for _, (lv, _) in diags.items() if lv >= 1]
        if not bad:
            return self.term.paint(f'진단 OK({len(diags)})', GREEN)
        return self.term.paint(f'진단 주의 {len(bad)}', RED + BOLD if max(bad) >= 2 else YELLOW)
