import pytest

from rl_correction.corrector_node import correct


def test_zero_command_stays_zero():
    assert correct((0.0, 0.0), (0.01, 0.01), 0.26, 1.82) == (0.0, 0.0)


def test_adds_and_clips():
    assert correct((0.2, 0.0), (0.01, -0.02), 0.26, 1.82) == pytest.approx((0.21, -0.02))
    assert correct((0.25, 1.8), (0.1, 0.5), 0.26, 1.82) == (0.26, 1.82)


def test_scale_action_clamps_then_scales():
    from rl_correction.corrector_node import scale_action
    assert scale_action([0.5, -2.0], 0.10, 0.60) == pytest.approx((0.05, -0.60))
    assert scale_action([3.0, 0.1], 0.10, 0.60) == pytest.approx((0.10, 0.06))
