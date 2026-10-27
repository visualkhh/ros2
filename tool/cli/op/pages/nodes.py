"""노드 목록 (같은 ROS_DOMAIN_ID에 보이는 것 전부)."""
from __future__ import annotations

from ..defs import STAY
from .base import Menu, MenuItem


class PrintNodesItem(MenuItem):
    key = '1'

    def label(self):
        return '목록 출력'

    def run(self):
        names = self.bus.node_names()
        print('\n'.join(names) if names else '노드 없음')
        return STAY


class NodesMenu(Menu):
    title = '노드'

    def items(self):
        return [PrintNodesItem(self.app)]

    def center(self):
        names = self.bus.node_names()
        return [f'[노드 {len(names)}개]'] + [f' {n}' for n in names] if names else [
            '노드 없음 — 베이스/SLAM/Nav가 꺼졌거나 ROS_DOMAIN_ID가 다름']
