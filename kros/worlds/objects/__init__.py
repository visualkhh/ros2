"""WorldObject(월드에 놓이는 모든 것의 베이스)와 로봇이 아닌 객체들."""
from .puck import Puck, PuckShape
from .world_object import WorldObject

__all__ = ["Puck", "PuckShape", "WorldObject"]
