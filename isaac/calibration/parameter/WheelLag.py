"""바퀴 모터 동역학 모델. 한 파일 한 클래스. (값이 아니라 plant다.)
`wheel_targets`(드라이버와 같은 v,w → 바퀴 식)도 여기 둔다 — calibration sim · 학습 env · learn 공용.

명령 v,w → (delay) → 바퀴 rad/s → (gain/deadband) → 1차 지연.
`CalibHUD`가 실행 중 tau/gain/deadband/delay를 직접 만지므로 필드는 그대로 둔다.
"""
from __future__ import annotations

import math
from collections import deque

import torch  # noqa: E402

from calibration.parameter.WheelParam import WheelParam
from urdf import robot_spec as spec


def wheel_targets(v, w, r, sep, max_wheel):
    """Twist → (좌, 우) rad/s. 한 바퀴라도 넘으면 두 바퀴를 같은 비율로 (opencr_driver.wheel_speeds와 같음)."""
    wl = (v - w * sep / 2.0) / r
    wr = (v + w * sep / 2.0) / r
    wheels = torch.stack([wl, wr], dim=-1)
    peak = wheels.abs().max(dim=-1, keepdim=True).values
    return wheels * torch.clamp(max_wheel / (peak + 1e-9), max=1.0)


class WheelLag:
    """`params`로 초기화. 이후 필드 직접 수정 가능 (라이브 튜닝용)."""

    def __init__(self, params: WheelParam, dt: float = 1.0 / spec.CONTROL_HZ):
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
