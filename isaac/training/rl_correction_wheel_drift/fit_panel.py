"""학습 중 랜덤 범위 보기 창 (`--watch` 전용, 읽기만).

랜덤 범위(실기기 맞춤 중심 포함) + 마찰 · 지형 설정을 보여준다.
학습 조건은 시작 때 고정이라 수정 필드는 두지 않는다.
"""
from __future__ import annotations



class FitPanel:
    """env cfg로 창을 만든다. 참조를 들고 있어야 창이 유지된다."""

    LABEL_STYLE = {"font_size": 13, "color": 0xFF9FB4C7}
    VALUE_STYLE = {"font_size": 13, "color": 0xFFF0F0F0}
    HEAD_STYLE = {"font_size": 12, "color": 0xFFF0C040}

    def __init__(self, env_cfg):
        import omni.ui as ui

        self.ui = ui
        c = env_cfg

        self.window = ui.Window("TRAINING RANGES", width=460, height=640)
        try:
            self.window.position_x, self.window.position_y = 18, 48
        except Exception:
            pass
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=3):
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
