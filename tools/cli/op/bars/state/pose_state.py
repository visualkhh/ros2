"""PoseState — 로봇의 지금 map 위치 칸 (amcl_pose). 없으면 '위치 -'."""
from __future__ import annotations

import math

from ...terminal import DIM
from .state import State


class PoseState(State):
    def text(self) -> str:
        p = self.state.amcl
        if p is None:
            return '위치 ' + self.term.paint('-', DIM)
        x, y, th = p
        return f'위치 x={x:+.2f} y={y:+.2f} yaw={math.degrees(th):+.0f}°'
