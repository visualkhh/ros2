import time
import unittest

from kros.hardwares.hardware import Hardware
from kros.worlds.clock import Clock


class Counter(Hardware):
    period = 0.1

    def __init__(self) -> None:
        super().__init__()
        self.calls = 0

    def on_period(self, clock: Clock) -> None:
        self.calls += 1


class TestPeriod(unittest.TestCase):
    def test_first_tick_with_epoch_time_does_not_loop_for_ages(self) -> None:
        """하드웨어 시각이 time.time()(epoch)이어도 첫 주기 계산이 폭주하지 않는다 (실기 kros 루프)."""
        counter = Counter()
        started = time.monotonic()
        counter.tick(Clock(1_700_000_000.0, 0.02))
        self.assertEqual(counter.calls, 1)
        self.assertLess(time.monotonic() - started, 0.5)
        counter.tick(Clock(1_700_000_000.05, 0.05))
        self.assertEqual(counter.calls, 1)
        counter.tick(Clock(1_700_000_000.11, 0.06))
        self.assertEqual(counter.calls, 2)


if __name__ == "__main__":
    unittest.main()
