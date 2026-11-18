"""PoseState — 로봇의 지금 map 위치 칸 (amcl_pose). 없으면 '위치 -'."""
from __future__ import annotations

import math

from ...terminal import DIM
from .state import State


class PoseState(State):
    def text(self) -> str:
        p = self.state.amcl
        age = self.state.amcl_rate.age()
        if p is None or age is None or age > 2.0:   # Nav를 끄면 마지막 amcl_pose가 남는다
            return '위치 ' + self.term.paint('-', DIM)
        x, y, th = p
        return f'위치 x={x:+.2f} y={y:+.2f} yaw={math.degrees(th):+.0f}°'
