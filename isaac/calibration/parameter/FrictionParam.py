"""마찰 값 (static, dynamic). 한 파일 한 클래스."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class FrictionParam:
    ground: tuple[float, float] = field(default_factory=lambda: (0.8, 0.7))
    obstacle: tuple[float, float] = field(default_factory=lambda: (0.7, 0.5))
    wheel: tuple[float, float] = field(default_factory=lambda: (1.0, 1.0))
    caster: tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))
