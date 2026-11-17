"""ConnectionState — /state/* 통신이 끊기면 상단에 빨강으로 표시한다."""
from __future__ import annotations

import time

from ...terminal import BOLD, GREEN, RED, YELLOW
from .state import State


class ConnectionState(State):
    def text(self) -> str:
        last = self.state.st_time
        if last is None:
            return self.term.paint('연결 ✗ 상태 없음 — bringup 확인', RED + BOLD)
        age = time.monotonic() - last
        if age <= 3.0:   # /state/* 하트비트 + 여유
            return self.term.paint(f'연결 ● 정상 {age:.1f}s', GREEN)
        if age <= 6.0:
            return self.term.paint(f'연결 ▲ {age:.1f}s 지연', YELLOW)
        return self.term.paint(f'연결 ✗ {age:.0f}s 끊김 — bringup 확인', RED + BOLD)
