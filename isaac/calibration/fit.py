"""fit.json ↔ 바퀴 모델 값. 표준 라이브러리만 (sim 띄우기 전에 불러 잘못된 경로면 바로 실패).

fit.json 모양: {"source": raw.csv, "plant": {...}?, "fit": {wheel_gain, wheel_gain_l, wheel_gain_r,
wheel_delay_steps, wheel_tau_s, wheel_deadband, ...}} — report.fit_wheel이 만든다.
"""
from __future__ import annotations

import json
from pathlib import Path

from urdf.robot_spec import CONTROL_HZ


def wheel_of(fit: dict) -> dict:
    """fit 평탄 dict → 바퀴 모델 값 {tau_s, delay_steps, gain_l, gain_r, deadband} (= WheelModelCfg 필드)."""
    g = float(fit.get("wheel_gain", 1.0))
    return {"tau_s": float(fit.get("wheel_tau_s", 0.0)),
            "delay_steps": int(fit.get("wheel_delay_steps", 0)),
            "gain_l": float(fit["wheel_gain_l"]) if fit.get("wheel_gain_l") is not None else g,
            "gain_r": float(fit["wheel_gain_r"]) if fit.get("wheel_gain_r") is not None else g,
            "deadband": float(fit.get("wheel_deadband", 0.0))}


def load_fit(spec: str) -> dict:
    """`--fit` 값 → sim 입력 보정 (fit 평탄 dict + source).

    `real/fit.json` 1개: real plant 절대값 그대로.
    `real/fit.json,sim/fit.json` 2개: sim plant 위에 얹을 차이 — gain은 real/sim 비율, delay/τ는 real−sim (음수면 0).
    deadband는 빼기가 아니다: 보정단은 sim gain 앞에 있으므로 real 문턱 ÷ sim gain, sim 문턱이 이미 더 크면 0.
    """
    paths = [str(Path(p).expanduser().resolve()) for p in spec.split(",") if p.strip()]
    if len(paths) not in (1, 2):
        raise ValueError(f"--fit은 fit.json 1개 또는 'real,sim' 2개: {spec}")
    real, *sim = [wheel_of(json.loads(Path(p).read_text())["fit"]) for p in paths]
    gl, gr = real["gain_l"], real["gain_r"]
    delay, tau, db = real["delay_steps"], real["tau_s"], real["deadband"]
    if sim:
        s = sim[0]
        # shortcut: 1차 지연 두 개 직렬 ≈ τ 합으로 근사, 정확히 하려면 2차 모델 fit
        gl, gr = gl / s["gain_l"], gr / s["gain_r"]
        delay, tau = max(0, delay - s["delay_steps"]), max(0.0, tau - s["tau_s"])
        db = db / ((s["gain_l"] + s["gain_r"]) / 2) if db > s["deadband"] else 0.0
    return {"wheel_gain": round((gl + gr) / 2, 4), "wheel_gain_l": round(gl, 4), "wheel_gain_r": round(gr, 4),
            "wheel_delay_steps": int(delay), "wheel_tau_s": round(tau, 4), "wheel_deadband": round(db, 4),
            "source": paths}


def describe(w: dict) -> str:
    return (f"tau={w['tau_s']:.3f}s delay={w['delay_steps'] / CONTROL_HZ:.2f}s "
            f"gain L/R={w['gain_l']:.3f}/{w['gain_r']:.3f} deadband={w['deadband']:.2f}")


if __name__ == "__main__":   # 자체 확인: 차이 계산
    import tempfile
    d = Path(tempfile.mkdtemp())
    (d / "r.json").write_text(json.dumps({"fit": {"wheel_gain_l": 0.96, "wheel_gain_r": 1.02, "wheel_delay_steps": 3,
                                                  "wheel_tau_s": 0.1, "wheel_deadband": 0.3}}))
    (d / "s.json").write_text(json.dumps({"fit": {"wheel_gain": 0.98, "wheel_delay_steps": 1, "wheel_tau_s": 0.02}}))
    f = load_fit(f"{d}/r.json,{d}/s.json")
    assert (f["wheel_gain_l"], f["wheel_gain_r"], f["wheel_delay_steps"], f["wheel_tau_s"]) == (0.9796, 1.0408, 2, 0.08), f
    assert abs(f["wheel_deadband"] - 0.3 / 0.98) < 1e-3
    assert wheel_of(load_fit(f"{d}/r.json"))["gain_l"] == 0.96
    print("ok")
