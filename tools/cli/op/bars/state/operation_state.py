"""OperationState — 조작 모드 칸 (MANUAL · NAV · SLAM_AUTO)."""
from __future__ import annotations

from ...terminal import BOLD, CYAN, GREEN, RED, YELLOW
from .state import State


class OperationState(State):
    """state_manager /state_control — 화면에는 Lv2처럼 짧은 이름으로."""
    LABELS = {'MANUAL': 'MANUAL', 'NAV_PLAN': 'NAV', 'SLAM_AUTO_MAPPING': 'SLAM_AUTO'}
    COLORS = {'MANUAL': YELLOW, 'NAV_PLAN': GREEN, 'SLAM_AUTO_MAPPING': CYAN}

    def text(self) -> str:
        mode = self.state.st_control
        if mode is None:
            return '조작 ' + self.term.paint('?', YELLOW)
        label = self.LABELS.get(mode, mode)
        return '조작 ' + self.term.paint(label, self.COLORS.get(mode, RED) + BOLD)
