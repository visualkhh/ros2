from tb3_bringup.opencr_driver_node import wheel_speeds

R, SEP, MAX = 0.033, 0.287, 8.0


def test_within_limit_unchanged():
    wl, wr = wheel_speeds(0.1, 0.0, R, SEP, MAX)
    assert abs(wl - 0.1 / R) < 1e-9 and abs(wr - 0.1 / R) < 1e-9


def test_fast_arc_scales_both_keeps_ratio():
    # 최고 속도 사선: 바깥 바퀴가 상한을 넘음 → 둘 다 같은 비율로 (안쪽 바퀴가 0으로 남아 제자리 회전이 되면 안 됨)
    raw_l, raw_r = wheel_speeds(0.26, 0.91, R, SEP, 0.0)
    wl, wr = wheel_speeds(0.26, 0.91, R, SEP, MAX)
    assert abs(max(abs(wl), abs(wr)) - MAX) < 1e-9
    assert abs(wl / wr - raw_l / raw_r) < 1e-9
    assert wl > 0  # 안쪽 바퀴도 앞으로
