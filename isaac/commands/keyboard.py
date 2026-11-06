"""KeyboardCommand — 사람 입력 (Isaac 창의 키 상태 폴링, carb.input). AppLauncher 뒤에 import.

W/A/S/D(화살표) 주행 · Shift = 0.3배 · X = 정지 · `[`/`]` = 속도 -/+. `pressed()`는 다른 키(R · F · C · N · P · L)를 쓰는 루프도 같이 쓴다.
"""
from __future__ import annotations

import carb.input
import omni.appwindow

from commands.base_command import BaseCommand

KEYS = ("W", "A", "S", "D", "UP", "DOWN", "LEFT", "RIGHT", "LEFT_SHIFT", "RIGHT_SHIFT", "X", "C", "R", "N", "P", "F", "L",
        "LEFT_BRACKET", "RIGHT_BRACKET")
_inp = carb.input.acquire_input_interface()
_kbd = omni.appwindow.get_default_app_window().get_keyboard()
_map = {n: getattr(carb.input.KeyboardInput, n) for n in KEYS}


def pressed() -> set:
    return {n for n, key in _map.items() if _inp.get_keyboard_button_flags(_kbd, key) & (1 << 3)}


def drive_cmd(keys: set, speed: float, turn: float, slow: float = 0.3) -> tuple[float, float]:
    """W/A/S/D(화살표) → (v, w). Shift = slow배 · X = 정지."""
    if "X" in keys:
        return 0.0, 0.0
    k = slow if keys & {"LEFT_SHIFT", "RIGHT_SHIFT"} else 1.0
    v = speed * k * (bool(keys & {"W", "UP"}) - bool(keys & {"S", "DOWN"}))
    w = turn * k * (bool(keys & {"A", "LEFT"}) - bool(keys & {"D", "RIGHT"}))
    return v, w


class KeyboardCommand(BaseCommand):
    def __call__(self, t: float) -> tuple[float, float]:
        return drive_cmd(pressed(), self.speed, self.turn)
