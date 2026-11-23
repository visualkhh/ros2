import math
import struct
import unittest

from kros.hardwares.lidars.lds_03 import _Parser

BEAMS = 360


def packet(ct: int, points: list[tuple[float, float]]) -> bytes:
    """COIN-D4 패킷을 만든다 (points: (각도 도, 거리 m)). 첫/끝 각도는 1/64도, bit0 체크비트."""
    count = len(points)
    first = int(round(points[0][0] * 64)) << 1 | 1
    last = int(round(points[-1][0] * 64)) << 1 | 1
    body = bytearray()
    for _, distance in points:
        mm = int(round(distance * 1000))
        b2, b1, b0 = mm >> 6, (mm & 0x3F) << 2, 0
        body += bytes((b0, b1, b2))
    checksum = 0x55AA ^ first ^ (ct | (count << 8)) ^ last
    for i in range(count):
        b0, b1, b2 = body[i * 3], body[i * 3 + 1], body[i * 3 + 2]
        checksum ^= b0
        checksum ^= b1 | (b2 << 8)
    return struct.pack("<HBBHHH", 0x55AA, ct, count, first, last, checksum & 0xFFFF) + bytes(body)


class TestLds03Parser(unittest.TestCase):
    def test_packet_becomes_a_scan(self) -> None:
        parser = _Parser(BEAMS)
        parser.feed(packet(0x01, [(0.0, 1.0), (90.0, 2.0), (180.0, 3.0)]))  # 첫 바퀴 시작
        parser.feed(packet(0x00, [(270.0, 4.0)]))  # 같은 바퀴
        parser.feed(packet(0x01, [(0.0, 0.5)]))  # 다음 바퀴 시작 -> 지난 스캔 확정
        scan = parser.take_scan()
        assert scan is not None
        self.assertEqual(len(scan), BEAMS)
        self.assertAlmostEqual(scan[0], 1.0)
        self.assertAlmostEqual(scan[90], 2.0)
        self.assertAlmostEqual(scan[180], 3.0)
        self.assertAlmostEqual(scan[270], 4.0)
        self.assertTrue(math.isinf(scan[45]))
        self.assertIsNone(parser.take_scan())  # 꺼내면 사라진다

    def test_keeps_the_closest_hit_per_beam(self) -> None:
        parser = _Parser(BEAMS)
        parser.feed(packet(0x01, [(10.0, 2.0), (10.2, 1.0)]))
        parser.feed(packet(0x01, [(0.0, 0.5)]))
        scan = parser.take_scan()
        assert scan is not None
        self.assertAlmostEqual(scan[10], 1.0)

    def test_bad_checksum_is_dropped(self) -> None:
        parser = _Parser(BEAMS)
        broken = bytearray(packet(0x01, [(0.0, 1.0)]))
        broken[-1] ^= 0xFF
        parser.feed(bytes(broken))
        parser.feed(packet(0x01, [(0.0, 0.5)]))
        self.assertIsNone(parser.take_scan())  # 깨진 패킷만으로는 스캔이 안 된다

    def test_noise_before_the_header_is_skipped(self) -> None:
        parser = _Parser(BEAMS)
        parser.feed(b"\x00\x01\x02" + packet(0x01, [(0.0, 1.0)]))
        parser.feed(packet(0x01, [(0.0, 0.5)]))
        scan = parser.take_scan()
        assert scan is not None
        self.assertAlmostEqual(scan[0], 1.0)

    def test_reverse_mirrors_the_angles(self) -> None:
        parser = _Parser(BEAMS, reverse=True)
        parser.feed(packet(0x01, [(0.0, 1.0), (90.0, 2.0)]))
        parser.feed(packet(0x01, [(0.0, 0.5)]))
        scan = parser.take_scan()
        assert scan is not None
        self.assertAlmostEqual(scan[0], 1.0)
        self.assertAlmostEqual(scan[270], 2.0)


if __name__ == "__main__":
    unittest.main()
