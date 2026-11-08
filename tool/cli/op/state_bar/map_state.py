"""MapState — map 수신 상태 + 지도 이름 (RateState 하위)."""
from __future__ import annotations

from ..terminal import DIM
from .rate_state import RateState


class MapState(RateState):
    def __init__(self, ui):
        super().__init__(ui, 'map', 'map')

    def text(self) -> str:
        txt = super().text()
        name = (self.state.st_map or '').strip()
        if name == '(SLAM 작성 중)':
            name = 'SLAM'
        if name:
            if len(name) > 20:
                name = name[:19] + '…'
            txt += ' ' + self.term.paint(name, DIM)
        return txt
