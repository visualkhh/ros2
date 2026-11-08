"""RateState — 토픽 수신 상태 한 칸 (✗ 없음 · 3초 넘게 끊김 노랑 · 수신 Hz 초록). odom·scan·map의 부모."""
from __future__ import annotations

from ..terminal import GREEN, RED, YELLOW
from .state import State


class RateState(State):
    """attr = StateStore의 속도 측정기 이름 (odom · scan · map)."""

    def __init__(self, ui, attr: str, label: str):
        super().__init__(ui)
        self.attr, self.label = attr, label

    def text(self) -> str:
        rate = getattr(self.state, self.attr)
        age = rate.age()
        if age is None:
            return self.term.paint(f'{self.label} ✗', RED)
        if age > 3.0:
            return self.term.paint(f'{self.label} {age:.0f}s', YELLOW)
        return self.term.paint(f'{self.label} {rate.hz():.0f}Hz', GREEN)
