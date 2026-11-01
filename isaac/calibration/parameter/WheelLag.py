"""바퀴 모터 동역학 모델. 한 파일 한 클래스. (값이 아니라 plant다.)

명령 v,w → (delay) → 바퀴 rad/s → (gain/deadband) → 1차 지연.
`CalibHUD`가 실행 중 tau/gain/deadband/delay를 직접 만지므로 필드는 그대로 둔다.
"""
from __future__ import annotations

import math
from collections import deque

import torch  # noqa: E402

from calibration.parameter.WheelParam import WheelParam


class WheelLag:
    """`params`로 초기화. 이후 필드 직접 수정 가능 (라이브 튜닝용)."""

    def __init__(self, params: WheelParam, dt: float = 0.05):
        self.dt = dt
        self.tau = float(params.tau_s)
        self.delay_steps = max(0, int(params.delay_steps))
        self.gain = float(params.gain)
        self.gain_l = params.gain_l
        self.gain_r = params.gain_r
        self.deadband = float(params.deadband)
        self._buf = deque([(0.0, 0.0)] * (self.delay_steps + 1), maxlen=self.delay_steps + 1)
        self._state: torch.Tensor | None = None

    def reset(self):
        self._buf = deque([(0.0, 0.0)] * (self.delay_steps + 1), maxlen=self.delay_steps + 1)
        self._state = None

    def set_delay_steps(self, steps: int):
        """실행 중 dead time 변경. 버퍼는 마지막 명령으로 다시 채운다."""
        steps = max(0, int(steps))
        if steps == self.delay_steps:
            return
        last = self._buf[-1] if self._buf else (0.0, 0.0)
        self.delay_steps = steps
        self._buf = deque([last] * (steps + 1), maxlen=steps + 1)

    def step(self, v: float, w: float, wheel_fn, r, sep, max_wheel, device):
        self._buf.append((float(v), float(w)))
        vd, wd = self._buf[0]
        wheels = wheel_fn(
            torch.tensor([vd], device=device),
            torch.tensor([wd], device=device),
            r, sep, max_wheel,
        )
        gl = self.gain if self.gain_l is None else self.gain_l
        gr = self.gain if self.gain_r is None else self.gain_r
        wheels = wheels * wheels.new_tensor([gl, gr])
        if self.deadband > 0:
            wheels = torch.where(wheels.abs() < self.deadband, torch.zeros_like(wheels), wheels)
        if self.tau <= 1e-6:          # tau=0 → 지연 없이 바로 반영
            self._state = wheels
        else:
            if self._state is None:
                self._state = wheels.clone()
            alpha = 1.0 - math.exp(-self.dt / self.tau)
            self._state = self._state + alpha * (wheels - self._state)
        return self._state
