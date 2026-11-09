"""TopStateBar — 화면 맨 위 상태바 기본 구성 (StateBar 하위 클래스)."""
from __future__ import annotations

from ...procs import SERVICES
from .domain_state import DomainState
from .goal_state import GoalState
from .map_state import MapState
from .proc_state import ProcState
from .rate_state import RateState
from .robot_state import RobotState
from .state_bar import StateBar


class TopStateBar(StateBar):
    """줄 1 프로세스(서비스마다) · 줄 2 odom·scan·map·목표 · 줄 3 로봇 · 줄 4 제어·운행·비상정지."""

    def __init__(self, ui):
        super().__init__(ui, [
            [ProcState(ui, name, svc.label) for name, svc in SERVICES.items()],
            [RateState(ui, 'odom', 'odom'), RateState(ui, 'scan', 'scan'), MapState(ui), GoalState(ui)],
            [RobotState(ui)],
            [DomainState(ui)],
        ])
