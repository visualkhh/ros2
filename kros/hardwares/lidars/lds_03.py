"""ROBOTIS LDS-03 (COIN-D4, M1CT_TOF) 2D 라이다. TurtleBot3 Waffle Pi 개조형에 USB2LDS(CP2102)로 꽂힌다.

시리얼 230400 baud, 패킷(COIN-D4 프로토콜):
  AA 55 | CT(1) | 샘플 수(1) | 시작각(2, LE, bit0=체크비트, 나머지 15비트가 1/64도) | 끝각(2) | 체크섬(2)
  샘플 3바이트 x N: 거리 = b2<<6 | b1>>2 (mm), 세기 = (b1&3)*64 + b0>>2, b0 bit0 = exp
  체크섬 = 0x55AA ^ 시작각 ^ 끝각 ^ (CT | N<<8) ^ 각 샘플의 (b0, b1|b2<<8)
CT bit0=1이면 한 바퀴의 시작이다: 그때까지 모은 점을 한 스캔으로 확정하고 새 스캔을 시작한다.

sample()은 시리얼 스레드가 마지막으로 완성한 스캔을 돌려준다. 점은 360칸으로 다시 나눈다 (칸마다 가장 가까운 거리).
포트가 없거나 끊기면 1초마다 다시 열고, 데이터가 3초 없으면 시작 명령(AA 55 F0 0F)을 다시 보낸다.
"""
from __future__ import annotations

import errno
import fcntl
import logging
import math
import os
import select
import struct
import termios
import threading
import time
from pathlib import Path

from kros.hardwares.lidars.lds_01 import Lds01
from kros.msgs.sensor_msgs import LaserScan
from kros.msgs.std_msgs import Header
from kros.worlds.clock import Clock

logger = logging.getLogger(__name__)

BAUD = 230400
BEAMS = 360
RANGE_MIN, RANGE_MAX = 0.05, 12.0  # LDS-03 사양 (0.05~12 m)
START = b"\xaa\x55\xf0\x0f"
STOP = b"\xaa\x55\xf5\x0a"
HEADER = b"\xaa\x55"
MAX_SAMPLES = 128


class Lds03(Lds01):
    """LDS-03 공통부: 360칸, 10Hz. 실제 스캔은 SerialLds03가 만든다."""

    model = "LDS-03"

    def __init__(self, mount_x: float = 0.060, mount_height: float = 0.022, period: float | None = 0.1, frame_id: str = "base_scan") -> None:
        super().__init__(mount_x, mount_height, period)
        self.frame_id = frame_id


class _Packet:
    """한 패킷의 점들 (각도 도, 거리 m)."""

    __slots__ = ("points", "ring_start")

    def __init__(self, points: list[tuple[float, float]], ring_start: bool) -> None:
        self.points = points
        self.ring_start = ring_start


class _Parser:
    """시리얼 바이트 -> 패킷. 순수 함수라 시리얼 없이 시험할 수 있다."""

    def __init__(self, beams: int = BEAMS, reverse: bool = False) -> None:
        self.beams = beams
        self.reverse = reverse
        self.buffer = bytearray()
        self.bad_packets = 0
        self.last_interval = 0.0  # 끝각 < 시작각일 때 쓸 지난 패킷의 각도 간격 (1/64도)
        self._pending: list[tuple[float, float]] = []
        self._scan: tuple[float, ...] | None = None
        self._lock = threading.Lock()

    def feed(self, data: bytes) -> None:
        self.buffer += data
        if len(self.buffer) > 65536:  # 헤더를 못 찾고 쌓이면 버린다
            self.buffer = self.buffer[-4096:]
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer = self.buffer[-1:]
                return
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 10:
                return
            header = bytes(self.buffer[:10])
            count = header[3]
            if count == 0 or count > MAX_SAMPLES:
                del self.buffer[0]
                self.bad_packets += 1
                continue
            size = 10 + count * 3
            if len(self.buffer) < size:
                return
            payload = bytes(self.buffer[10:size])
            del self.buffer[:size]
            packet = self._parse(header, payload, count)
            if packet is None:
                self.bad_packets += 1
                continue
            if packet.ring_start and self._pending:  # 새 바퀴 시작: 지난 스캔을 확정한다
                self._complete()
            self._pending.extend(packet.points)

    def _complete(self) -> None:
        bins = [math.inf] * self.beams
        for angle, distance in self._pending:
            if not math.isfinite(distance):
                continue
            index = int(angle / 360.0 * self.beams) % self.beams
            if distance < bins[index]:
                bins[index] = distance
        with self._lock:
            self._scan = tuple(bins)
        self._pending = []

    def _parse(self, header: bytes, payload: bytes, count: int) -> _Packet | None:
        ct = header[2]
        first_raw = header[4] | (header[5] << 8)
        last_raw = header[6] | (header[7] << 8)
        checksum = header[8] | (header[9] << 8)
        calculated = 0x55AA ^ first_raw ^ (ct | (count << 8)) ^ last_raw
        for i in range(count):
            b0, b1, b2 = payload[i * 3], payload[i * 3 + 1], payload[i * 3 + 2]
            calculated ^= b0
            calculated ^= b1 | (b2 << 8)
        if calculated & 0xFFFF != checksum:
            return None
        if first_raw & 0x01 == 0 or last_raw & 0x01 == 0:  # 체크비트가 없으면 깨진 패킷
            return None
        first, last = first_raw >> 1, last_raw >> 1
        if count == 1:
            interval = 0.0
        elif last < first:
            if first > 270 * 64 and last < 90 * 64:
                interval = (360 * 64 + last - first) / (count - 1)
            else:
                interval = self.last_interval
        else:
            interval = (last - first) / (count - 1)
            self.last_interval = interval
        points: list[tuple[float, float]] = []
        for i in range(count):
            b0, b1, b2 = payload[i * 3], payload[i * 3 + 1], payload[i * 3 + 2]
            distance = ((b2 << 6) | (b1 >> 2)) / 1000.0  # mm -> m
            angle = (first + i * interval) / 64.0  # 1/64도 -> 도
            if self.reverse:
                angle = 360.0 - angle
            angle %= 360.0
            if RANGE_MIN <= distance <= RANGE_MAX:
                points.append((angle, distance))
        return _Packet(points, bool(ct & 0x01))

    def take_scan(self) -> tuple[float, ...] | None:
        with self._lock:
            scan, self._scan = self._scan, None
            return scan


class _SerialThread(threading.Thread):
    """시리얼 포트를 열고 바이트를 parser로 넘긴다. 끊기면 다시 연다."""

    def __init__(self, port: str, baud: int, parser: _Parser, reconnect_s: float = 1.0) -> None:
        super().__init__(daemon=True, name=f"lds03:{port}")
        self.port = port
        self.baud = baud
        self.parser = parser
        self.reconnect_s = reconnect_s
        self._stop = threading.Event()
        self._fd: int | None = None
        self._write_lock = threading.Lock()
        self._warned = False
        self.last_data = 0.0

    def stop(self) -> None:
        self._stop.set()
        self._close()

    def _close(self) -> None:
        with self._write_lock:
            fd, self._fd = self._fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def write(self, data: bytes) -> bool:
        with self._write_lock:
            fd = self._fd
            if fd is None:
                return False
            try:
                os.write(fd, data)
                return True
            except OSError:
                return False

    def _open(self) -> bool:
        try:
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as error:
            if not self._warned:
                logger.warning("%s를 열지 못했다: %s (%.0f초마다 다시 시도)", self.port, error, self.reconnect_s)
                self._warned = True
            return False
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = getattr(termios, f"B{self.baud}")
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        try:  # USB2LDS는 DTR을 내려야 데이터를 보내기 시작한다 (coin_d4_driver와 같다)
            fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR))
        except OSError:
            pass
        with self._write_lock:
            self._fd = fd
        self._warned = False
        logger.info("%s 연결 (%d baud)", self.port, self.baud)
        time.sleep(0.1)
        self.write(START)
        self.last_data = time.monotonic()
        return True

    def run(self) -> None:
        while not self._stop.is_set():
            if self._fd is None and not self._open():
                self._stop.wait(self.reconnect_s)
                continue
            fd = self._fd
            if fd is None:
                continue
            try:
                if not select.select([fd], [], [], 0.1)[0]:
                    if time.monotonic() - self.last_data > 3.0:  # 데이터가 없으면 시작 명령을 다시
                        self.write(START)
                        self.last_data = time.monotonic()
                    continue
                data = os.read(fd, 4096)
            except OSError as error:
                if error.errno != errno.EAGAIN:
                    logger.warning("%s 읽기 실패: %s (다시 연결한다)", self.port, error)
                    self._close()
                continue
            if data:
                self.last_data = time.monotonic()
                self.parser.feed(data)


class SerialLds03(Lds03):
    """진짜 LDS-03: 시리얼 스레드가 스캔을 만들고, sample()이 마지막 스캔을 LaserScan으로 낸다.

    body_filter: 로봇 몸통(1층 판·바퀴·지지대)이 찍힌 점을 지운다. 라이다가 앞(x=0.060)에 있어 뒤쪽 반이 몸통이다.
    상자는 로봇 발자국 전체(base_link 기준 x -0.21~0.075, y ±0.17)를 덮는다: 라이다 자체/판 모서리(5cm)가
    충돌 모니터의 전방 상자에 걸려 텔레옵이 막히는 것을 막는다. 이 안의 점은 실제 장애물일 수 없다.
    """

    BODY_BOX = (-0.21, 0.075, -0.17, 0.17)

    def __init__(
        self,
        port: str = "",
        mount_x: float = 0.060,
        mount_height: float = 0.022,
        period: float | None = 0.1,
        frame_id: str = "base_scan",
        beams: int = BEAMS,
        reverse: bool = False,
        body_filter: bool = True,
    ) -> None:
        super().__init__(mount_x, mount_height, period, frame_id)
        chosen = port or ("/dev/tb3_lidar" if Path("/dev/tb3_lidar").exists() else "/dev/ttyUSB0")
        self.parser = _Parser(beams, reverse)
        self.body_filter = body_filter
        self._thread = _SerialThread(chosen, BAUD, self.parser)
        self._thread.start()
        self.scan_count = 0

    @property
    def port(self) -> str:
        return self._thread.port

    @property
    def connected(self) -> bool:
        return self._thread._fd is not None

    def sample(self, clock: Clock) -> LaserScan | None:
        ranges = self.parser.take_scan()
        if ranges is None:
            return None
        self.scan_count += 1
        increment = 2 * math.pi / self.parser.beams
        if self.body_filter:
            min_x, max_x, min_y, max_y = self.BODY_BOX
            filtered = list(ranges)
            for i, distance in enumerate(ranges):
                if not math.isfinite(distance):
                    continue
                angle = i * increment
                x = self.mount_x + distance * math.cos(angle)
                y = distance * math.sin(angle)
                if min_x <= x <= max_x and min_y <= y <= max_y:
                    filtered[i] = math.inf
            ranges = tuple(filtered)
        return LaserScan(Header(clock.time, self.frame_id), 0.0, 2 * math.pi - increment, increment, RANGE_MIN, RANGE_MAX, ranges)

    def stop(self) -> None:
        self._thread.write(STOP)
        self._thread.stop()
