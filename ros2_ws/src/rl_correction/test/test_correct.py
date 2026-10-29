import pytest

from rl_correction.corrector_node import correct


def test_zero_command_stays_zero():
    assert correct((0.0, 0.0), (0.01, 0.01), 0.26, 1.82) == (0.0, 0.0)


def test_adds_and_clips():
    assert correct((0.2, 0.0), (0.01, -0.02), 0.26, 1.82) == pytest.approx((0.21, -0.02))
    assert correct((0.25, 1.8), (0.1, 0.5), 0.26, 1.82) == (0.26, 1.82)
