"""학습 중 보정값 보기 창 (`--watch` 전용, 읽기만). 한 파일 한 클래스.

`--fit` fit.json 내용 + 그걸로 적용된 랜덤 범위 + 마찰 설정을 보여준다.
학습 조건은 시작 때 고정이므로 `CalibHUD`(calibration/random_world.py)와 달리
수정 필드는 두지 않는다.
"""
from __future__ import annotations

import json
from pathlib import Path


class FitPanel:
    """env cfg(적용 후)와 fit.json 경로로 창을 만든다. 참조를 들고 있어야 창이 유지된다."""

    LABEL_STYLE = {"font_size": 13, "color": 0xFF9FB4C7}
    VALUE_STYLE = {"font_size": 13, "color": 0xFFF0F0F0}
    HEAD_STYLE = {"font_size": 12, "color": 0xFFF0C040}

    def __init__(self, env_cfg, fit_path: str | None, latency: dict | None = None):
        import omni.ui as ui

        self.ui = ui
        c = env_cfg
        fit = json.loads(Path(fit_path).read_text())["fit"] if fit_path else None

        self.window = ui.Window("TRAINING FIT", width=460, height=640)
        try:
            self.window.position_x, self.window.position_y = 18, 48
        except Exception:
            pass
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=3):
                    self._heading("FIT (--fit)")
                    if fit is None:
                        self._row("source", "없음 — 기본 범위")
                    else:
                        p = Path(fit_path)
                        power = fit.get("power") or {}
                        self._row("source", f"{p.parent.name}/{p.name}")
                        self._row("gain L / R", self._pair(fit.get("wheel_gain_l"), fit.get("wheel_gain_r")))
                        self._row("delay", f"{fit.get('wheel_delay_steps')} step ({fit.get('wheel_delay_s')} s)")
                        self._row("tau [s]", fit.get("wheel_tau_s"))
                        self._row("deadband [rad/s]", fit.get("wheel_deadband"))
                        self._heading("FIT — 학습에 안 씀 (참고)")
                        self._row("deadband L / R", self._pair(fit.get("wheel_deadband_l"), fit.get("wheel_deadband_r")))
                        self._row("power straight L / R", self._pair(power.get("straight_l"), power.get("straight_r")))
                        self._row("power turn L / R", self._pair(power.get("turn_l"), power.get("turn_r")))
                        self._row("wheel max obs [rad/s]", fit.get("wheel_max_obs"))
                        self._row("saturated samples", fit.get("saturated_samples"))

                    self._heading("RL 경로 지연 (--latency)")
                    if latency is None:
                        self._row("source", "없음 — 기본 지연 범위")
                    else:
                        self._row("source", Path(latency["source"]).name)
                        self._row("관측 나이 / 처리 [ms]", self._pair(latency["obs_age_ms"], latency["proc_ms"]))
                        self._row("관측 지연 / 명령 추가 [step]",
                                  self._pair(latency["obs_delay_steps"], latency["cmd_extra_steps"]))

                    self._heading("RANGES (에피소드마다 랜덤)")
                    self._row("gain center L / R", self._pair(*c.gain_center))
                    self._row("gain range (× center)", self._range(c.gain_range))
                    self._row("tau [s]", self._range(c.tau_range))
                    self._row("cmd delay [step]", self._range(c.cmd_delay_steps))
                    self._row("obs delay [step]", self._range(c.obs_delay_steps))
                    self._row("deadband [rad/s]", self._range(c.deadband_range))

                    self._heading("FRICTION (static / dynamic)")
                    self._row("ground", f"{self._pair(*c.ground_friction)} (고정)")
                    if c.events is not None:
                        p = c.events.wheel_friction.params
                        self._row("wheel (env마다 랜덤)", f"{self._range(p['static_friction_range'])} / "
                                                         f"{self._range(p['dynamic_friction_range'])}")
                    self._row("wheel", self._pair(*c.wheel_friction))
                    self._row("caster", self._pair(*c.caster_friction))
                    self._row("terrain", f"경사 0~{c.slope_deg:g}° · 요철 {'on' if c.uneven else 'off'}")

    @staticmethod
    def _v(x) -> str:
        return "—" if x is None else str(x)   # fit에서 못 잰 값

    @classmethod
    def _pair(cls, a, b) -> str:
        return f"{cls._v(a)} / {cls._v(b)}"

    @staticmethod
    def _range(r) -> str:
        return f"{r[0]} ~ {r[1]}"

    def _heading(self, text):
        self.ui.Label(text, height=22, style=self.HEAD_STYLE)

    def _row(self, label, value):
        with self.ui.HStack(height=20):
            self.ui.Label(label, width=180, style=self.LABEL_STYLE)
            self.ui.Label(self._v(value), style=self.VALUE_STYLE)
