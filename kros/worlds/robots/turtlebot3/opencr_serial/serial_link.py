import logging
import os
import select
import termios
import threading
from collections.abc import Callable

logger = logging.getLogger(__name__)


class SerialLink:
    """시리얼 포트(USB CDC, /dev/opencr) 하나로 줄을 주고받는다. 읽기 스레드가 줄이 완성될 때마다 on_line(줄)을 부르고,
    포트가 없거나 끊기면 reconnect_s초마다 다시 연다 (포트가 없어도 start()는 실패하지 않는다). 표준 라이브러리(termios)만 쓴다.

    on_line은 읽기 스레드에서 불린다.
    """

    def __init__(self, port: str, baud: int, on_line: Callable[[str], None], reconnect_s: float = 1.0) -> None:
        self.port, self.baud, self._on_line, self._reconnect_s = port, baud, on_line, reconnect_s
        self._fd: int | None = None
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return self._fd is not None

    def start(self) -> None:
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name=f"serial:{self.port}", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self._close()

    def write_line(self, line: str) -> bool:
        """한 줄을 보낸다. 연결이 안 됐거나 쓰다 실패하면 False."""
        with self._write_lock:
            fd = self._fd
            if fd is None:
                return False
            try:
                os.write(fd, (line + "\n").encode())
            except OSError as error:
                logger.warning("%s 쓰기 실패: %s", self.port, error)
                return False
            return True

    def _open(self) -> bool:
        try:
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as error:
            if not self._warned:
                logger.warning("%s를 열지 못했다: %s (%.0f초마다 다시 시도한다)", self.port, error, self._reconnect_s)
                self._warned = True
            return False
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0  # raw: iflag, oflag, lflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = getattr(termios, f"B{self.baud}")
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self._fd = fd
        self._warned = False
        logger.info("%s 연결", self.port)
        return True

    _warned = False

    def _close(self) -> None:
        with self._write_lock:
            fd, self._fd = self._fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _run(self) -> None:
        buffer = b""
        while not self._stop.is_set():
            if self._fd is None and not self._open():
                self._stop.wait(self._reconnect_s)
                continue
            fd = self._fd
            if fd is None:
                continue
            try:
                if not select.select([fd], [], [], 0.1)[0]:
                    continue
                data = os.read(fd, 4096)
            except OSError as error:
                logger.warning("%s 읽기 실패: %s (끊긴 것으로 보고 다시 연결한다)", self.port, error)
                self._close()
                buffer = b""
                continue
            buffer += data
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self._on_line(line)
            if len(buffer) > 1024:  # 줄바꿈 없이 쌓이면 깨진 것이다
                buffer = b""
