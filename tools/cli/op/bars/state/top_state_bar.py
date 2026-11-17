"""TopStateBar — 화면 맨 위 상태바 기본 구성 (StateBar 하위 클래스)."""
from __future__ import annotations

from ...procs import SERVICES
from .collision_state import CollisionState
from .connection_state import ConnectionState
from .controller_state import ControllerState
from .diag_state import DiagState
from .estop_state import EstopState
from .goal_state import GoalState
from .map_state import MapState
from .operation_state import OperationState
from .pose_state import PoseState
from .power_state import PowerState
from .proc_state import ProcState
from .rate_state import RateState
from .robot_state import RobotState
from .state_bar import StateBar


class TopStateBar(StateBar):
    """줄 1 프로세스(서비스마다) · 줄 2 조작·지도·위치·E-STOP·충돌감시·연결 · 줄 3 odom·스캔·amcl·목표·주행·진단 · 줄 4 모터·전원."""

    def __init__(self, ui):
        super().__init__(ui, [
            [ProcState(ui, name, svc.label) for name, svc in SERVICES.items()],
            [OperationState(ui), MapState(ui), PoseState(ui), EstopState(ui),
             CollisionState(ui), ConnectionState(ui)],
            [RateState(ui, 'odom', 'odom'), RateState(ui, 'scan', '스캔'), RateState(ui, 'amcl_rate', 'amcl'),
             GoalState(ui), ControllerState(ui), DiagState(ui)],
            [RobotState(ui), PowerState(ui)],
        ])
