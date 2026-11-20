from collections.abc import Collection
from dataclasses import dataclass

from kros.interactions.interaction import Interaction


@dataclass(frozen=True)
class KeyState:
    """지금 눌려 있는 키들 (w, a, s, d, up, down, left, right, q, z ...)."""

    keys: frozenset[str]


class Keyboard(Interaction[KeyState]):
    """키보드: 눌린 키 집합이 바뀔 때만 입력을 내보낸다 (진짜 키보드가 이벤트를 보내는 것처럼)."""

    def set_keys(self, keys: Collection[str]) -> None:
        state = KeyState(frozenset(keys))
        if state != self.state:
            self.feed(state)
