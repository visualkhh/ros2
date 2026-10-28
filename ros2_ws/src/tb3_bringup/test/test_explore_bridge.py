from tb3_bringup.explore_bridge_node import MODE, decide


def test_not_automapping_pauses_running_explore():
    # explore_lite는 켜지자마자 탐색 시작 → MANUAL이면 바로 멈춰야
    assert decide('MANUAL', 'exploration_started', 0.0, 15.0) == 'pause'
    assert decide('MANUAL', 'exploration_complete', 99.0, 15.0) is None


def test_complete_right_after_start_retries():
    # SLAM 막 켠 빈 지도 — frontier 없음으로 바로 complete
    assert decide(MODE, 'exploration_complete', 3.0, 15.0) == 'resume'


def test_complete_after_grace_goes_manual():
    assert decide(MODE, 'exploration_complete', 120.0, 15.0) == 'manual'
    assert decide(MODE, 'exploration_in_progress', 120.0, 15.0) is None
