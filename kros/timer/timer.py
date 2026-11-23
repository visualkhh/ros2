from collections.abc import Callable


class Timer:
    """주기 호출 하나의 손잡이. 콜백은 이 타이머 전용 워커 스레드 하나에서 순서대로 돈다 (예외는 로그만 남기고 타이머는 계속 돈다).

    사실만 노출한다: calls(콜백이 실제로 불린 횟수), missed(주기가 됐지만 부르지 못한 횟수: 이전 콜백이 아직 실행 중이었거나 시간이 한꺼번에 건너뛰었다).
    """

    def __init__(self, period: float, callback: Callable[[], None], cancel: Callable[["Timer"], None], name: str = "") -> None:
        self.period = period
        self.name = name  # 만든 노드 이름 (프로파일/디버그용)
        self.callback = callback
        self.calls = 0
        self.missed = 0
        self.is_canceled = False
        self._cancel = cancel

    def cancel(self) -> None:
        """타이머를 멈춘다 (여러 번 불러도 안전)."""
        self._cancel(self)

    def __repr__(self) -> str:
        return f"Timer(period={self.period}, calls={self.calls}, missed={self.missed}, canceled={self.is_canceled})"
