"""바퀴 모터 값. 한 파일 한 클래스."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class WheelParam:
    """1차 지연 τ + transport delay + 좌/우 gain + 데드밴드."""

    tau_s: float = 0.05
    delay_steps: int = 2        # 20 Hz 기준 (×0.05 s)
    gain: float = 0.994         # 공통 gain (좌/우 개별값이 없을 때 대체)
    gain_l: float | None = None  # 왼쪽 gain (None이면 gain 사용)
    gain_r: float | None = None  # 오른쪽 gain (None이면 gain 사용)
    deadband: float = 0.0       # [rad/s]

    def gains(self) -> tuple[float, float]:
        """(왼쪽, 오른쪽) gain. 개별값이 없으면 공통 gain으로 채운다."""
        gl = self.gain if self.gain_l is None else self.gain_l
        gr = self.gain if self.gain_r is None else self.gain_r
        return float(gl), float(gr)
