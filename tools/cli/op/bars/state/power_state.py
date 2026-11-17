"""PowerState — 전원 칸 (OpenCR 진단의 배터리 전압·잔량)."""
from __future__ import annotations

from ...terminal import GREEN, YELLOW
from .state import State


class PowerState(State):
    def text(self) -> str:
        st = self.state
        age = st.robot.age()
        if age is None or age > 3.0:
            return ''
        vals = st.robot_vals
        try:
            bat = float(vals['battery_V'])
            pct = float(vals['battery_pct'])
        except (KeyError, ValueError):
            return ''
        low = bat < 11.0
        return '전원 ' + self.term.paint(
            f'{"▲" if low else "●"} 배터리 {bat:.1f}V({pct:.0f}%)', YELLOW if low else GREEN)
