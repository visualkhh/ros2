"""장면 계약. 한 파일 한 클래스.

구현은 `DefaultScene`이 들고, 방 구조·마찰·로봇 배치가 다르면 서브클래스가
메서드를 오버라이딩한다. 직접 인스턴스화하지 않는다.
"""
from __future__ import annotations

from abc import ABC, abstractmethod


class Scene(ABC):
    """방 + 로봇 + 카메라 조립기. build → spawn → settle → place 순으로 쓴다."""

    TAG = "scene"
    WALL_H = 1.1
    WALL_T = 0.05
    ROBOT_Z = 0.03
    ROOM_SPACING = 8.0      # 방 사이 [m] — 라이다(≈3.5 m)가 옆방을 못 보게
    FREE_RADIUS = 0.75      # 로봇 스폰 자리 — 장애물 금지
    CAM_OFF = (1.8, -1.8, 1.55)
    CAM_LOOK_Z = 0.15

    PALETTE = (
        (0.75, 0.15, 0.12), (0.12, 0.55, 0.20), (0.15, 0.30, 0.75),
        (0.75, 0.55, 0.10), (0.45, 0.20, 0.60), (0.20, 0.60, 0.65),
    )

    def __init__(self, condition: str = "raw", actuator=None):
        self.condition = condition
        self.actuator = actuator   # ActuatorParam — None이면 SimParam(condition)의 값 (damping 측정 때 덮어씀)
        self.rooms = []
        self.scene = None
        self.robot = None
        self.usd_path: str | None = None

    @abstractmethod
    def build(self, count: int = 1, size: float = 4.0, seed: int = 0, size_range=None,
              condition: str | None = None):
        """바닥·조명·방 count개. 반환: [Room]. count=0이면 빈 바닥만."""
        raise NotImplementedError

    @abstractmethod
    def set_camera(self, sim, origin=(0.0, 0.0, 0.0)):
        raise NotImplementedError

    @abstractmethod
    def articulation_cfg(self, prim_path: str = "/World/Robot", usd_path: str | None = None,
                         fix_root: bool = False, init_z: float | None = None,
                         condition: str | None = None):
        """로봇 ArticulationCfg + USD 경로."""
        raise NotImplementedError

    @abstractmethod
    def bind_robot_materials(self, scene=None, prim_path: str = "/World/Robot",
                             condition: str | None = None):
        """바퀴·캐스터 마찰을 sim_calib_<condition>.json에서."""
        raise NotImplementedError

    @abstractmethod
    def spawn(self, airborne: bool = False, init_z: float | None = None,
              condition: str | None = None):
        """우리 URDF → USD → /World/Robot. 반환: (scene, robot, usd_path)."""
        raise NotImplementedError

    @abstractmethod
    def place(self, origin, yaw: float = 0.0, wheel_ids=None, robot=None):
        """로봇을 방 origin으로 순간이동 (속도·바퀴 상태 0)."""
        raise NotImplementedError

    @abstractmethod
    def settle(self, sim, steps: int = 120, scene=None, robot=None):
        """스폰 높이에서 떨어져 바퀴·캐스터로 서게. 1초."""
        raise NotImplementedError
