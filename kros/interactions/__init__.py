"""사람이 넣는 입력 장치 (Interaction). 로봇이 receive()로 받는다."""
from .interaction import Interaction
from .keyboards import Keyboard, KeyState

__all__ = ["Interaction", "KeyState", "Keyboard"]
