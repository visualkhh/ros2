"""GoalState — Nav2 목표 진행 상태 (목표가 없으면 빈 칸)."""
from __future__ import annotations

from .state import State


class GoalState(State):
    def text(self) -> str:
        goal = self.state.goal_status
        return f'목표 {goal}' if goal else ''
