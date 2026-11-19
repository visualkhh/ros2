"""LocalizationState(위치추정 방식: MAPPING / LOCALIZATION / NONE) · MapState(적용된 지도 이름) 칸 — 둘 다 Tab으로 골라 Enter로 바꾼다."""
from __future__ import annotations

from ...terminal import DIM, GREEN, YELLOW
from .state import State


class LocalizationState(State):
    """state_manager /state_operation — SLAM이면 MAPPING(지도 작성하며 위치추정), Nav면 LOCALIZATION(저장된 지도에서 AMCL)."""
    COLORS = {'MAPPING': YELLOW, 'LOCALIZATION': GREEN, 'NONE': DIM}
    focusable = True

    def activate(self):
        return self.ui.window.navigate('localization')

    def text(self) -> str:
        op = self.state.st_operation
        if op is None:
            return '위치추정: ' + self.term.paint('-', DIM)
        mode = {'SLAM': 'MAPPING', 'LOCALIZATION': 'LOCALIZATION'}.get(op, 'NONE')
        return '위치추정: ' + self.term.paint(mode, self.COLORS.get(mode, DIM))


class MapState(State):
    focusable = True

    def activate(self):
        return self.ui.window.navigate('map-change')

    def text(self) -> str:
        name = (self.state.st_map or '').strip()
        if name == '(SLAM 작성 중)':
            name = 'SLAM'
        if not name:
            return '지도: ' + self.term.paint('-', DIM)
        return '지도: ' + self.term.paint(name if len(name) <= 20 else name[:19] + '…', DIM)
