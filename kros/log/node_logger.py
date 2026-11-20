import logging
import sys
from collections.abc import Callable

from kros.msgs.rcl_interfaces import Log, LogLevel

PYTHON_LEVELS = {
    LogLevel.DEBUG: logging.DEBUG,
    LogLevel.INFO: logging.INFO,
    LogLevel.WARN: logging.WARNING,
    LogLevel.ERROR: logging.ERROR,
    LogLevel.FATAL: logging.CRITICAL,
}


class NodeLogger:
    """노드의 로거 (ROS의 get_logger()). 한 번 부르면 두 곳으로 나간다: 파이썬 logging(로거 이름 "kros.node.<노드이름>")과
    rosout 토픽(Log 메시지: kros 시간, 수준, 노드 이름, 메시지, 부른 곳의 파일/함수/줄). 수준 미만은 어디에도 안 나간다.

    메시지는 logging처럼 `%` 인자를 받는다: logger.info("속도 %.2f", v). 포매팅이 틀려도 예외를 내지 않는다 (로그가 프로그램을 죽이면 안 된다).
    주의: rosout을 구독하는 콜백 안에서 로그를 남기면 그 로그가 다시 rosout으로 나가 끝없이 이어진다 (ROS도 같다).
    """

    def __init__(self, node_name: str, publish: Callable[[Log], None], now: Callable[[], float], level: Callable[[], LogLevel]) -> None:
        self.name = node_name
        self._publish = publish
        self._now = now
        self._level = level  # 지금 수준을 돌려주는 함수 (log_level 파라미터를 따라 실행 중에도 바뀐다)
        self._python = logging.getLogger(f"kros.node.{node_name}")

    def debug(self, msg: str, *args: object) -> None:
        self._log(LogLevel.DEBUG, msg, args)

    def info(self, msg: str, *args: object) -> None:
        self._log(LogLevel.INFO, msg, args)

    def warn(self, msg: str, *args: object) -> None:
        self._log(LogLevel.WARN, msg, args)

    def error(self, msg: str, *args: object) -> None:
        self._log(LogLevel.ERROR, msg, args)

    def fatal(self, msg: str, *args: object) -> None:
        self._log(LogLevel.FATAL, msg, args)

    def is_enabled_for(self, level: LogLevel) -> bool:
        """이 수준의 로그가 나가는가 (메시지를 만드는 비용이 큰 곳에서 먼저 확인한다)."""
        return level >= self._level()

    def _log(self, level: LogLevel, msg: str, args: tuple[object, ...]) -> None:
        if level < self._level():
            return
        if args:
            try:
                msg = msg % args
            except (TypeError, ValueError):
                msg = f"{msg} (포매팅 실패: {args!r})"
        frame = sys._getframe(2)  # debug/info/... 를 부른 곳
        code = frame.f_code
        self._python.log(PYTHON_LEVELS[level], msg, stacklevel=3)
        self._publish(Log(self._now(), level, self.name, msg, code.co_filename, code.co_name, frame.f_lineno))


CONSOLE_HANDLER_NAME = "kros-console"


def enable_console(level: int = logging.DEBUG) -> logging.Handler:
    """kros 노드 로그를 콘솔(stderr)에 보이게 한다 ("[LEVEL] [노드]: 메시지"). 여러 번 불러도 핸들러는 하나다.
    라이브러리가 멋대로 켜지 않도록 기본은 꺼져 있고, 프로그램이 원할 때 부른다 (수준은 노드의 log_level 파라미터가 따로 정한다)."""
    root = logging.getLogger("kros.node")
    for handler in root.handlers:
        if handler.get_name() == CONSOLE_HANDLER_NAME:
            handler.setLevel(level)
            return handler
    handler = logging.StreamHandler()
    handler.set_name(CONSOLE_HANDLER_NAME)
    handler.setLevel(level)
    handler.setFormatter(logging.Formatter("[%(levelname)s] [%(name)s]: %(message)s"))
    root.addHandler(handler)
    root.setLevel(logging.DEBUG)  # 걸러내기는 노드의 log_level이 한다
    return handler
