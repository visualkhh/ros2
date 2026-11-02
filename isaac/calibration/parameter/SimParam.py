"""sim 보정값 묶음. 한 파일 한 클래스.

`artifacts/sim_calib_<condition>.json` (plot_compare가 real CSV에서 뽑아 저장)을
읽어 타입 있는 값으로 준다. 파일이 없으면 실측 기본값 (2026-10-30).
`condition="raw"`는 보정 없는 기준 — sim 실행기들의 기본 (보정은 `--fit`으로만 넣는다).
JSON 스키마는 손대지 않는다 (plot_compare가 직접 읽고 쓴다).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from calibration.parameter.ActuatorParam import ActuatorParam
from calibration.parameter.FrictionParam import FrictionParam
from calibration.parameter.WheelParam import WheelParam

ISAAC = Path(__file__).resolve().parents[2]
CALIB_DIR = ISAAC / "artifacts"


def calib_path(condition: str) -> Path:
    return CALIB_DIR / f"sim_calib_{condition}.json"


@dataclass
class SimParam:
    """호출부는 dict 키 문자열 대신 이 타입을 받는다."""

    wheel: WheelParam = field(default_factory=WheelParam)
    actuator: ActuatorParam = field(default_factory=ActuatorParam)
    friction: FrictionParam = field(default_factory=FrictionParam)
    source: str = "defaults"

    @classmethod
    def defaults(cls) -> "SimParam":
        return cls()

    @classmethod
    def raw(cls) -> "SimParam":
        """보정 없는 기준 — 이전 JSON/기본값을 무시하고 plant만 그대로 본다."""
        return cls(wheel=WheelParam(tau_s=0.0, delay_steps=0, gain=1.0, deadband=0.0),
                   source="raw")

    @classmethod
    def from_dict(cls, d: dict) -> "SimParam":
        """plot_compare가 쓰는 평탄 JSON → 타입. 모르는 키는 무시한다."""
        return cls(
            wheel=WheelParam(
                tau_s=float(d.get("wheel_tau_s", 0.05)),
                delay_steps=int(d.get("wheel_delay_steps", 2)),
                gain=float(d.get("wheel_gain", 0.994)),
                gain_l=None if d.get("wheel_gain_l") is None else float(d["wheel_gain_l"]),
                gain_r=None if d.get("wheel_gain_r") is None else float(d["wheel_gain_r"]),
                deadband=float(d.get("wheel_deadband", 0.0)),
            ),
            actuator=ActuatorParam(
                damping=float(d.get("actuator_damping", 50.0)),
                effort=float(d.get("actuator_effort", 3.0)),
            ),
            friction=FrictionParam(
                ground=(float(d["friction_ground"][0]), float(d["friction_ground"][1])) if "friction_ground" in d else (0.8, 0.7),
                obstacle=(float(d["friction_obstacle"][0]), float(d["friction_obstacle"][1])) if "friction_obstacle" in d else (0.7, 0.5),
                wheel=(float(d["friction_wheel"][0]), float(d["friction_wheel"][1])) if "friction_wheel" in d else (1.0, 1.0),
                caster=(float(d["friction_caster"][0]), float(d["friction_caster"][1])) if "friction_caster" in d else (0.0, 0.0),
            ),
            source=str(d.get("source", "defaults")),
        )

    @classmethod
    def load(cls, condition: str = "floor") -> "SimParam":
        if condition == "raw":
            return cls.raw()
        try:
            d = json.loads(calib_path(condition).read_text())
        except Exception:
            d = {}
        base = cls.defaults()
        if not d:
            return base
        keep = cls.from_dict({**{
            "wheel_tau_s": base.wheel.tau_s,
            "wheel_delay_steps": base.wheel.delay_steps,
            "wheel_gain": base.wheel.gain,
            "wheel_gain_l": base.wheel.gain_l,
            "wheel_gain_r": base.wheel.gain_r,
            "wheel_deadband": base.wheel.deadband,
            "actuator_damping": base.actuator.damping,
            "actuator_effort": base.actuator.effort,
            "friction_ground": list(base.friction.ground),
            "friction_obstacle": list(base.friction.obstacle),
            "friction_wheel": list(base.friction.wheel),
            "friction_caster": list(base.friction.caster),
            "source": base.source,
        }, **d})
        return keep

    def describe(self) -> str:
        w = self.wheel
        gl, gr = w.gains()
        gains = f"gain={w.gain:.3f}" if (w.gain_l is None and w.gain_r is None) \
            else f"gain L/R={gl:.3f}/{gr:.3f}"
        return (f"tau={w.tau_s:.3f}s "
                f"delay={w.delay_steps * 0.05:.2f}s "
                f"{gains} deadband={w.deadband:.2f} "
                f"src={self.source}")
