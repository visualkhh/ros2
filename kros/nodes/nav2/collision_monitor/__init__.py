"""nav2 collision_monitor 흉내: 라이다 점과 로봇 둘레 영역으로 속도 명령을 막거나 늦춘다 (마지막 안전장치)."""
from .collision_monitor import ACTIONS, CollisionMonitor, Polygon

__all__ = ["ACTIONS", "CollisionMonitor", "Polygon"]
