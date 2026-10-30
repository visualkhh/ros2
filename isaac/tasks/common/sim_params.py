"""sim-real 보정값 — sim 쪽 파라미터를 real 측정에 맞춘다 (2026-10-30).

실측 근거:
- 공중(지지 박스): v_enc 0.150 vs 0.149, 바퀴 4.545 vs 4.518 (gain 0.994)
- real 계단: t10 0.10 s (dead time), t50 0.15, t90 0.20
  → dead 0.1 s + 1차 지연 τ≈0.05 s (10→90% ≈ 0.1 s)
- 바닥: sim 바퀴가 부하에서 4.23/3.90으로 처짐 → 액추에이터 damping을 올려 real(4.52)에 맞춤

바꾸면 여기만 고친다. world viewer(01) · 센서 싱크(02)가 같이 쓴다.
RL env는 학습 전에 같은 값을 import한다.
"""
from __future__ import annotations

import math
from collections import deque

import torch

# --- 모터/명령 모델 ---
WHEEL_TAU = 0.05          # [s] 1차 지연 (10→90% ≈ 0.11 s)
WHEEL_DELAY_STEPS = 2     # 20 Hz 기준 0.1 s dead time (WiFi + 20 Hz)
WHEEL_GAIN = 0.994        # 정상상태 gain (real 4.518 / cmd 4.545)
WHEEL_DEADBAND = 0.0      # [rad/s] 아직 미측정 — 저속 시험 후 조정

# --- 액추에이터 (ImplicitActuatorCfg) ---
ACTUATOR_DAMPING = 100.0  # real 속도제어처럼 부하에서도 속도 유지 (초기값, 바닥 비교로 조정)
ACTUATOR_EFFORT = 3.0     # XM430-W210 stall ≈ 3 N·m

# --- 마찰 (static, dynamic) ---
FRICTION_GROUND = (0.8, 0.7)
FRICTION_OBSTACLE = (0.7, 0.5)
FRICTION_WHEEL = (1.0, 1.0)
FRICTION_CASTER = (0.0, 0.0)


class WheelLag:
    """명령 v,w → (delay) → 바퀴 rad/s → (gain/deadband) → 1차 지연.

    `wheel_fn(v_tensor, w_tensor, r, sep, max_wheel)`은 rl_correct의 wheel_targets.
    """

    def __init__(self, dt: float = 0.05, tau: float = WHEEL_TAU,
                 delay_steps: int = WHEEL_DELAY_STEPS, gain: float = WHEEL_GAIN,
                 deadband: float = WHEEL_DEADBAND):
        self.dt = dt
        self.tau = tau
        self.delay_steps = max(0, int(delay_steps))
        self.gain = gain
        self.deadband = deadband
        self._buf = deque([(0.0, 0.0)] * (self.delay_steps + 1), maxlen=self.delay_steps + 1)
        self._state: torch.Tensor | None = None

    def reset(self):
        self._buf = deque([(0.0, 0.0)] * (self.delay_steps + 1), maxlen=self.delay_steps + 1)
        self._state = None

    def step(self, v: float, w: float, wheel_fn, r, sep, max_wheel, device):
        self._buf.append((float(v), float(w)))
        vd, wd = self._buf[0]
        wheels = wheel_fn(
            torch.tensor([vd], device=device),
            torch.tensor([wd], device=device),
            r, sep, max_wheel,
        ) * self.gain
        if self.deadband > 0:
            wheels = torch.where(wheels.abs() < self.deadband, torch.zeros_like(wheels), wheels)
        if self._state is None:
            self._state = wheels.clone()
        alpha = 1.0 - math.exp(-self.dt / self.tau)
        self._state = self._state + alpha * (wheels - self._state)
        return self._state


def describe() -> str:
    return (f"tau={WHEEL_TAU:.3f}s delay={WHEEL_DELAY_STEPS * 0.05:.2f}s "
            f"gain={WHEEL_GAIN:.3f} deadband={WHEEL_DEADBAND:.2f} "
            f"damping={ACTUATOR_DAMPING:.0f}")
