"""실행 단위 계약. 한 파일 한 클래스.

sim 컨텍스트 수명주기만 들고, 무엇을 띄우고 어떻게 도는지는 서브클래스가
`run()`으로 채운다. 직접 인스턴스화하지 않는다.
"""
from __future__ import annotations

import sys
from abc import ABC, abstractmethod
from pathlib import Path

WS = Path(__file__).resolve().parents[3]          # ros2/
ISAAC = WS / "isaac"
if str(ISAAC) not in sys.path:
    sys.path.insert(0, str(ISAAC))

class World(ABC):
    """Isaac sim 실행 단위. build/run/close 순으로 쓴다."""

    TAG = "world"
    DT = 1 / 120.0

    def __init__(self, device: str = "cpu"):
        # pxr은 AppLauncher 뒤에만 import한다.
        from isaaclab.sim import SimulationCfg, SimulationContext
        self.sim = SimulationContext(SimulationCfg(dt=self.DT, device=device))

    @abstractmethod
    def run(self) -> None:
        """메인 루프. 서브클래스가 정의한다."""
        raise NotImplementedError

    def close(self) -> None:
        self.sim.stop()
