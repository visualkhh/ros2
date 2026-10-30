"""sim-real 보정값 — sim 쪽 파라미터를 real 측정에 맞춘다.

파일 (plot_compare가 real CSV에서 뽑아 저장):
  steps/artifacts/sim_calib_stand.json
  steps/artifacts/sim_calib_floor.json

사용:
  - 01_random_world      → floor
  - 02_sensor_sync --stand → stand
  - 02_sensor_sync (바닥)  → floor

파일이 없으면 아래 기본값 (2026-10-30 실측).
"""
from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path

import torch

ISAAC = Path(__file__).resolve().parents[2]
CALIB_DIR = ISAAC / "steps" / "artifacts"

DEFAULTS = {
    "wheel_tau_s": 0.05,
    "wheel_delay_steps": 2,
    "wheel_gain": 0.994,
    "wheel_deadband": 0.0,
    "actuator_damping": 100.0,
    "actuator_effort": 3.0,
    "friction_ground": (0.8, 0.7),
    "friction_obstacle": (0.7, 0.5),
    "friction_wheel": (1.0, 1.0),
    "friction_caster": (0.0, 0.0),
}


def calib_path(condition: str) -> Path:
    return CALIB_DIR / f"sim_calib_{condition}.json"


def values(condition: str = "floor") -> dict:
    d = dict(DEFAULTS)
    try:
        d.update(json.loads(calib_path(condition).read_text()))
    except Exception:
        pass
    return d


def actuator(condition: str = "floor"):
    d = values(condition)
    return float(d["actuator_damping"]), float(d["actuator_effort"])


def friction(condition: str = "floor"):
    d = values(condition)
    return {
        "ground": tuple(d["friction_ground"]),
        "obstacle": tuple(d["friction_obstacle"]),
        "wheel": tuple(d["friction_wheel"]),
        "caster": tuple(d["friction_caster"]),
    }


class WheelLag:
    """명령 v,w → (delay) → 바퀴 rad/s → (gain/deadband) → 1차 지연."""

    def __init__(self, condition: str = "floor", dt: float = 0.05,
                 tau: float | None = None, delay_steps: int | None = None,
                 gain: float | None = None, deadband: float | None = None):
        d = values(condition)
        self.dt = dt
        self.tau = float(d["wheel_tau_s"] if tau is None else tau)
        self.delay_steps = max(0, int(d["wheel_delay_steps"] if delay_steps is None else delay_steps))
        self.gain = float(d["wheel_gain"] if gain is None else gain)
        self.deadband = float(d["wheel_deadband"] if deadband is None else deadband)
        self.source = d.get("source", "defaults")
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
        ) * self.gain
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


def describe(condition: str = "floor") -> str:
    d = values(condition)
    return (f"tau={float(d['wheel_tau_s']):.3f}s "
            f"delay={int(d['wheel_delay_steps']) * 0.05:.2f}s "
            f"gain={float(d['wheel_gain']):.3f} deadband={float(d['wheel_deadband']):.2f} "
            f"damping={float(d['actuator_damping']):.0f} src={d.get('source', 'defaults')}")
