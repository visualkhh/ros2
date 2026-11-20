from __future__ import annotations

from abc import ABC
from collections.abc import Callable
from typing import Generic, TypeVar

T = TypeVar("T")


class Interaction(ABC, Generic[T]):
    """사람(조종자)이 넣는 입력 장치의 베이스 (키보드, 게임패드, 마우스 ...). kros와 무관한 순수 클래스.

    입력을 받아 상태(state)로 들고 있고, 받는 쪽(로봇)은 subscribe로 입력이 바뀔 때마다 알림을 받는다.
    입력을 넣는 쪽(3D 창, 진짜 장치 드라이버 ...)은 feed()를 부른다. 입력을 무슨 뜻으로 해석할지는 받는 쪽이 정한다.
    """

    def __init__(self) -> None:
        self.state: T | None = None  # 가장 최근 입력
        self._subscribers: list[Callable[[T], None]] = []

    def subscribe(self, callback: Callable[[T], None]) -> None:
        """입력이 들어올 때마다 callback(상태)가 불린다."""
        self._subscribers.append(callback)

    def feed(self, state: T) -> None:
        """입력을 넣는다 (입력 장치/창이 부른다)."""
        self.state = state
        for callback in self._subscribers:
            callback(state)
