"""ControllerState — 주행 제어기 RL/DWB 칸 (controller_server FollowPath 상태)."""
from __future__ import annotations

import time

from ...terminal import GREEN, YELLOW
from .state import State


def controller_text(st) -> str:
    """/controller_server/FollowPath/status → 'RL' · 'DWB(점구름/스캔 없음·stale)' — 2 s 넘게 없으면 ''."""
    from ...pages.nodes.controller_server import INNER, REASONS
    s = st.rl_status
    if not s or time.monotonic() - st.rl_status_time > 2.0:
        return ''
    if s[0] >= 0.5:
        return 'RL'
    inner = INNER.get(int(s[9]), '기본') if len(s) > 9 else '기본'
    why = REASONS.get(int(s[1]), '') if len(s) > 1 else ''
    return f'{inner}({why})' if why else inner


class ControllerState(State):
    def text(self) -> str:
        t = controller_text(self.state)
        return f'주행 {self.term.paint(t, GREEN if t == "RL" else YELLOW)}' if t else ''
