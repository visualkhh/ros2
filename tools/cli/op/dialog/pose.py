"""fmt_pose — 지도 화면·확인창에서 쓰는 (x, y, yaw°) 한 줄 표기."""
from __future__ import annotations


def fmt_pose(p: dict | None) -> str:
    return '지금 위치 (실행할 때)' if p is None else f'({p["x"]:+.2f}, {p["y"]:+.2f}) {p["yaw_deg"]:+.0f}°'
