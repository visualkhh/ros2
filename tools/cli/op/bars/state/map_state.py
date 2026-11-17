"""MapState — 지도 모드(MAPPING / LOCALIZATION / NONE) + 적용된 지도 이름 칸."""
from __future__ import annotations

from ...terminal import DIM, GREEN, YELLOW
from .state import State


class MapState(State):
    COLORS = {'MAPPING': YELLOW, 'LOCALIZATION': GREEN, 'NONE': DIM}

    def text(self) -> str:
        op = self.state.st_operation
        if op is None:
            return '지도: ' + self.term.paint('-', DIM)
        mode = {'SLAM': 'MAPPING', 'LOCALIZATION': 'LOCALIZATION'}.get(op, 'NONE')
        txt = '지도: ' + self.term.paint(mode, self.COLORS.get(mode, DIM))
        name = (self.state.st_map or '').strip()
        if name == '(SLAM 작성 중)':
            name = 'SLAM'
        if name:
            txt += ' ' + self.term.paint(name if len(name) <= 20 else name[:19] + '…', DIM)
        return txt
