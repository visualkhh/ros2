"""키보드 배열(한글/영문)과 상관없이 물리 키의 눌림을 읽는다. Panda3D의 키 이벤트는 현재 배열의 글자라서 한글 배열이면 w가 ㅈ으로 와서 조종 키가 안 먹는다.
X11(XQueryKeymap)의 물리 키 번호는 배열과 무관하다. X11을 못 쓰는 환경(윈도우, 맥, 라이브러리 없음)이면 pressed()가 None이라 이벤트 방식으로 남는다."""
import ctypes
import ctypes.util

KEYCODES = {"q": 24, "w": 25, "a": 38, "s": 39, "d": 40, "f": 41, "l": 46, "z": 52, "c": 54, "up": 111, "left": 113, "right": 114, "down": 116}  # X11(evdev) 키 번호


class PhysicalKeys:
    def __init__(self) -> None:
        self._x11: ctypes.CDLL | None = None
        self._display: ctypes.c_void_p | None = None
        name = ctypes.util.find_library("X11")
        if name:
            try:
                x11 = ctypes.CDLL(name)
                x11.XOpenDisplay.restype = ctypes.c_void_p
                x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
                x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char * 32]
                display = x11.XOpenDisplay(None)
                if display:
                    self._x11, self._display = x11, ctypes.c_void_p(display)
            except OSError:
                pass

    @property
    def available(self) -> bool:
        return self._x11 is not None

    def pressed(self) -> set[str] | None:
        """지금 눌려 있는 키 이름들 (조종 키 + 토글 키 f/c/l, 배열과 무관). 읽을 수 없으면 None."""
        if self._x11 is None:
            return None
        buffer = (ctypes.c_char * 32)()
        self._x11.XQueryKeymap(self._display, buffer)
        bits = bytes(buffer)
        return {name for name, code in KEYCODES.items() if bits[code // 8] >> (code % 8) & 1}

    def close(self) -> None:
        if self._x11 is not None:
            self._x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
            self._x11.XCloseDisplay(self._display)
            self._x11 = None
