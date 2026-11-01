"""방 한 칸. 한 파일 한 클래스."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Room:
    index: int
    origin: tuple[float, float, float]
    seed: int
    obstacles: int
    size: float
