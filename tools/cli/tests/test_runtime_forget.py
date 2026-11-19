#!/usr/bin/env python3
"""런타임 저장값 시험 — ROS 없이: python3 tools/cli/tests/test_runtime_forget.py

예전 문제(2026-11-19): 어제 끈 충돌 감시가 저장돼 있어서, Nav를 새로 켜도 저장값이 다시 적용돼 꺼진 채 시작했다.
확인: ① start(nav) 는 저장값을 적용하지 않고 지운다 ② restart 는 저장값을 다시 적용한다 ③ 지우면 화면 상태(override)도 비움
④ Procs.clear_collision_enabled 는 그 키만 지운다 (FollowPath.enable 은 그대로)
"""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
import geometry_msgs.msg as _gm  # noqa: E402
for _n in ('PoseStamped', 'PoseWithCovarianceStamped', 'Twist', 'TwistStamped'):   # 가짜 ROS에 없는 메시지 — import 만 되게
    if not hasattr(_gm, _n):
        setattr(_gm, _n, type(_n, (), {}))
import op.bus as busmod  # noqa: E402
from op.bus import Bus  # noqa: E402
from op.procs import Procs  # noqa: E402


def fake_bus():
    calls = []
    state = NS(collision_enabled=False, collision_override=False, rl_enabled=False)
    procs = NS(running=lambda n: False, start=lambda *a: (True, 'ok'), restart=lambda n: (True, 'ok'),
               clear_enable=lambda: calls.append('clear_enable'), clear_collision_enabled=lambda: calls.append('clear_collision'))
    bus = NS(state=state, procs=procs, term=NS(paint=lambda t, c: t), ui=NS(console=NS(isatty=lambda: False)),
             base_ready=lambda: True, _nav_init_hint=lambda: None,
             apply_saved_enable=lambda: calls.append('apply_enable'), apply_saved_collision=lambda: calls.append('apply_collision'))
    bus.forget_saved_runtime = lambda: Bus.forget_saved_runtime(bus)
    return bus, calls, state


bus, calls, state = fake_bus()
Bus.start(bus, 'nav', '/maps/a.yaml')
assert calls == ['clear_enable', 'clear_collision'], calls                      # ① 새로 켬 = 지움 · 적용 없음
assert (state.collision_enabled, state.collision_override, state.rl_enabled) == (None, None, None)   # ③

busmod.IS_ROBOT = True   # 재시작 적용은 로봇에서만 (ROS 서비스 호출)
bus, calls, _ = fake_bus()
Bus.restart(bus, 'nav')
assert calls == ['apply_enable', 'apply_collision'], calls                       # ② 재시작 = 저장값 적용

data = {'controller_server': {'FollowPath.enable': False}, 'collision_monitor': {'enabled': False}}
saved = []
p = NS(runtime_params=lambda: data, _save_runtime_params=lambda d: saved.append(dict(d)))
ok, _ = Procs.clear_collision_enabled(p)
assert ok and saved == [{'controller_server': {'FollowPath.enable': False}}]     # ④ 충돌감시만 지움
print('PASS test_runtime_forget')
