import math
import os
import pty
import select
import time
import unittest
from collections.abc import Callable

from kros.worlds.robots.turtlebot3.opencr_serial import OpenCrSerial, protocol
from kros.hardwares.controllers.diff_drive import wheel_speeds
from kros.msgs.geometry_msgs import Twist, Vector3
from kros.worlds.clock import Clock

NAN = "nan"
ARM_ABSENT = ",".join([NAN] * 18 + ["-1"] * 6)
S_LINE = "S,1.5,0.5,2.5,0.6,12345,0.1,0.2,30,31,11.8"
I_LINE = "I,0.01,0.02,0.3,0.1,0.2,9.8,0.9,0.1,0.2,0.3"


def eventually(check: Callable[[], bool], seconds: float = 2.0) -> bool:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.01)
    return check()


class Firmware:
    """가짜 펌웨어: 가상 시리얼(pty)의 반대쪽 끝. 진짜 /dev/opencr 대신 이 쪽을 보드가 연다 (같은 termios 경로를 탄다)."""

    def __init__(self) -> None:
        self.master, slave = pty.openpty()
        self.port = os.ttyname(slave)
        self.slave = slave  # 닫으면 포트가 사라지니 테스트가 끝날 때까지 들고 있는다
        self.received = b""

    def send(self, *lines: str) -> None:
        os.write(self.master, ("\n".join(lines) + "\n").encode())

    def lines(self) -> list[str]:
        while select.select([self.master], [], [], 0)[0]:
            self.received += os.read(self.master, 4096)
        return self.received.decode().splitlines()

    def close(self) -> None:
        os.close(self.master)
        os.close(self.slave)


class TestProtocol(unittest.TestCase):
    def test_status_line(self) -> None:
        kind, values = protocol.split_line(S_LINE) or ("", [])
        status = protocol.parse_status(values)
        assert status is not None
        self.assertEqual((kind, status.left_position, status.right_velocity, status.uptime_ms, status.battery_voltage), ("S", 1.5, 0.6, 12345, 11.8))

    def test_older_status_without_current_and_battery_still_parses(self) -> None:
        status = protocol.parse_status([1.0, 2.0, 3.0, 4.0, 100.0])
        assert status is not None
        self.assertTrue(math.isnan(status.battery_voltage))
        self.assertIsNone(protocol.parse_status([1.0, 2.0]))

    def test_imu_line_maps_wxyz_to_a_quaternion(self) -> None:
        values = (protocol.split_line(I_LINE) or ("", []))[1]
        imu = protocol.parse_imu(values)
        assert imu is not None
        self.assertEqual((imu.orientation.w, imu.orientation.x, imu.orientation.z), (0.9, 0.1, 0.3))  # 펌웨어 순서는 qw qx qy qz
        self.assertEqual((imu.angular_velocity.z, imu.linear_acceleration.z, imu.header.frame_id), (0.3, 9.8, "imu_link"))

    def test_garbage_is_not_a_line(self) -> None:
        self.assertIsNone(protocol.split_line("S,1,abc"))
        self.assertEqual(protocol.split_line("Q")  , ("Q", []))

    def test_error_text(self) -> None:
        self.assertEqual((protocol.error_text(0), protocol.error_text(-1), protocol.error_text(36)), ("정상", "무응답", "과열+과부하"))

    def test_arm_is_absent_when_everything_is_nan(self) -> None:
        values = (protocol.split_line("M," + ARM_ABSENT) or ("", []))[1]
        self.assertIs(protocol.arm_connected(values), False)
        self.assertIs(protocol.arm_connected([0.0] * 24), True)
        self.assertIsNone(protocol.arm_connected([0.0]))  # 짧은 줄은 모르겠다

    def test_command_line(self) -> None:
        self.assertEqual(protocol.format_command(1.0, -0.5), "C,1.0000,-0.5000")


class TestWheelSpeeds(unittest.TestCase):
    def test_straight_and_pivot(self) -> None:
        self.assertEqual(wheel_speeds(Twist(Vector3(x=0.33)), 0.033, 0.288), (10.0, 10.0))
        left, right = wheel_speeds(Twist(angular=Vector3(z=1.0)), 0.033, 0.288)
        self.assertAlmostEqual(left, -right)

    def test_limit_scales_both_wheels_to_keep_the_arc(self) -> None:
        free = wheel_speeds(Twist(Vector3(x=0.5), Vector3(z=2.0)), 0.033, 0.288)
        capped = wheel_speeds(Twist(Vector3(x=0.5), Vector3(z=2.0)), 0.033, 0.288, 8.0)
        self.assertAlmostEqual(max(map(abs, capped)), 8.0)
        self.assertAlmostEqual(capped[0] / capped[1], free[0] / free[1])  # 비율(호의 모양)이 같다


class TestOpenCrSerial(unittest.TestCase):
    def setUp(self) -> None:
        self.firmware = Firmware()
        self.board = OpenCrSerial(0.033, 0.288, port=self.firmware.port)
        self.addCleanup(self.firmware.close)
        self.board.start()
        self.addCleanup(self.board.stop)
        self.assertTrue(eventually(lambda: self.board.connected))

    def test_it_is_the_opencr_board_with_wheels_and_an_imu(self) -> None:
        self.assertEqual((self.board.model, self.board.parts), ("OpenCR1.0", [self.board.left, self.board.right, self.board.imu]))

    def test_status_line_updates_the_wheels_and_the_board_status(self) -> None:
        self.firmware.send(S_LINE)
        self.assertTrue(eventually(lambda: self.board.status is not None))
        self.assertEqual((self.board.left.position, self.board.left.velocity, self.board.right.position, self.board.right.velocity), (1.5, 0.5, 2.5, 0.6))
        assert self.board.status is not None
        self.assertEqual(self.board.status.battery_voltage, 11.8)

    def test_imu_value_is_stamped_with_the_clock_and_given_out_once(self) -> None:
        self.firmware.send(I_LINE)
        values: list[float] = []
        self.board.imu.subscribe(lambda m: values.append(m.header.stamp))
        self.assertTrue(eventually(lambda: self._tick_and_check(values)))
        self.assertEqual(values[0], 1.0)
        n = len(values)
        self._tick(2.0)
        self._tick(3.0)
        self.assertEqual(len(values), n)  # 새 I 줄이 없으면 같은 값을 다시 내지 않는다

    def _tick_and_check(self, values: list[float]) -> bool:
        self._tick(1.0)
        return bool(values)

    def _tick(self, t: float) -> None:
        self.board.tick(Clock(t, 0.1))  # 값이 올 때까지 같은 시각으로 계속 두드려도 새 값은 한 번만 나간다

    def test_the_command_is_sent_as_a_c_line_every_period(self) -> None:
        self.board.write(Twist(Vector3(x=0.2)))  # 0.2 / 0.033 = 6.0606 rad/s (한계 8 이내)
        self._tick(1.0)
        self.assertTrue(eventually(lambda: "C,6.0606,6.0606" in self.firmware.lines()))
        self._tick(1.2)
        self.assertTrue(eventually(lambda: self.firmware.lines().count("C,6.0606,6.0606") == 2))

    def test_a_command_over_the_firmware_limit_is_scaled_not_clipped_per_wheel(self) -> None:
        self.board.write(Twist(Vector3(x=0.5), Vector3(z=2.0)))
        self._tick(1.0)
        self.assertTrue(eventually(lambda: any(line.startswith("C,") for line in self.firmware.lines())))
        left, right = (float(v) for v in self.firmware.lines()[0].split(",")[1:])
        self.assertAlmostEqual(max(abs(left), abs(right)), 8.0, places=3)

    def test_stop_sends_zero_speed(self) -> None:
        self.board.write(Twist(Vector3(x=0.2)))
        self.board.stop()
        self.assertIn("C,0.0000,0.0000", self.firmware.lines())

    def test_wheel_error_is_recorded_once_per_change_not_every_repeat(self) -> None:
        self.firmware.send("E,0,36", "E,0,36", "E,0,36")
        self.assertTrue(eventually(lambda: self.board.right.error_count >= 1))
        time.sleep(0.1)
        self.assertEqual((self.board.left.error_count, self.board.right.error_count), (0, 1))
        self.assertIn("과열+과부하", str(self.board.right.last_error))
        self.firmware.send("E,0,0", "E,8,0")
        self.assertTrue(eventually(lambda: self.board.left.error_count == 1))

    def test_arm_is_only_noticed_not_driven(self) -> None:
        self.assertIsNone(self.board.arm_connected)
        self.firmware.send("M," + ARM_ABSENT)
        self.assertTrue(eventually(lambda: self.board.arm_connected is False))

    def test_unknown_and_broken_lines_do_not_stop_the_reader(self) -> None:
        self.firmware.send("Z,1,2", "S,1,abc", "garbage", S_LINE)
        self.assertTrue(eventually(lambda: self.board.status is not None))
        self.assertGreaterEqual(self.board.bad_lines, 1)  # 못 읽은 줄은 센다 (모르는 종류는 안 센다)


class TestOpenCrSerialDirection(unittest.TestCase):
    def test_inverted_motors_flip_commands_and_encoder_signs_together(self) -> None:
        """이 로봇은 두 모터 모두 +속도 = 뒤로 (robot.yaml left_dir/right_dir = -1). 명령과 엔코더에 같은 부호를 준다."""
        firmware = Firmware()
        board = OpenCrSerial(0.033, 0.288, port=firmware.port, left_dir=-1.0, right_dir=-1.0)
        self.addCleanup(firmware.close)
        board.start()
        self.addCleanup(board.stop)
        self.assertTrue(eventually(lambda: board.connected))
        board.write(Twist(Vector3(x=0.2)))
        board.tick(Clock(1.0, 0.1))
        self.assertTrue(eventually(lambda: "C,-6.0606,-6.0606" in firmware.lines()))
        firmware.send(S_LINE)
        self.assertTrue(eventually(lambda: board.status is not None))
        self.assertEqual((board.left.position, board.right.position), (-1.5, -2.5))


class TestCommandTimeout(unittest.TestCase):
    def test_last_command_is_dropped_when_writes_stop(self) -> None:
        """0.5초 동안 새 cmd_vel이 없으면 마지막 속도를 계속 보내지 않고 0으로 세운다 (네트워크가 끊겨도 로봇이 달리지 않게)."""
        firmware = Firmware()
        now = [0.0]
        board = OpenCrSerial(0.033, 0.288, port=firmware.port, command_timeout=0.5, now=lambda: now[0])
        self.addCleanup(firmware.close)
        board.start()
        self.addCleanup(board.stop)
        self.assertTrue(eventually(lambda: board.connected))
        board.write(Twist(Vector3(x=0.2)))
        now[0] = 0.1
        board.tick(Clock(1.0, 0.1))
        self.assertTrue(eventually(lambda: "C,6.0606,6.0606" in firmware.lines()))
        now[0] = 0.7  # 0.6초 지남: 새 명령 없음
        board.tick(Clock(1.1, 0.1))
        self.assertTrue(eventually(lambda: "C,0.0000,0.0000" in firmware.lines()))


class TestNoPort(unittest.TestCase):
    def test_it_starts_without_the_port_and_records_the_send_failure_once(self) -> None:
        board = OpenCrSerial(0.033, 0.288, port="/dev/does-not-exist")
        board.start()
        self.addCleanup(board.stop)
        for t in (1.0, 1.1, 1.2):
            board.tick(Clock(t, 0.1))
        self.assertFalse(board.connected)
        self.assertEqual(board.error_count, 1)  # 끊긴 사실은 한 번만


if __name__ == "__main__":
    unittest.main()
