#!/usr/bin/env python3
"""Bus.wait_until 시험 — ROS 없이: python3 tools/cli/tests/test_wait_until.py

오래 걸리는 ROS 대기는 전부 이 도우미를 쓴다 (예전엔 곳곳에 같은 while 루프를 복사해 문구·갱신이 제각각이었다).
확인: ① 이미 참이면 바로 True (poll 없음) ② 중간에 참이 되면 True · 끝까지 안 되면 False ③ 기다리는 동안 처리 중 창에
'what … N/M초'를 갱신 ④ what이 비면 아무것도 안 씀 ⑤ 명령줄(처리 중 창 없음)에서는 5초 넘게 걸릴 때만 print
⑥ wait_future · wait_server 는 같은 도우미 ⑦ 옛 직접 대기 루프가 bus.py 에 남지 않음 (속도를 계속 보내는 cmd_drive · spin_once 루프만 예외)
"""
import re
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
import geometry_msgs.msg as _gm  # noqa: E402
for _n in ('PoseStamped', 'PoseWithCovarianceStamped', 'Twist', 'TwistStamped'):   # 가짜 ROS에 없는 메시지 — import 만 되게
    if not hasattr(_gm, _n):
        setattr(_gm, _n, type(_n, (), {}))
from op.bus import Bus  # noqa: E402


def make(busy=True):
    notes, polls = [], []
    bus = NS(ui=NS(progress=notes.append, _busy=object() if busy else None), _poll=lambda s=0.2: (polls.append(s), time.sleep(s)))
    bus.wait_until = lambda *a, **k: Bus.wait_until(bus, *a, **k)
    return bus, notes, polls


bus, notes, polls = make()
assert Bus.wait_until(bus, lambda: True, 5, '준비') is True and polls == [] and notes == []          # ①

state = {'n': 0}
bus, notes, polls = make()
def third():
    state['n'] += 1
    return state['n'] >= 3
assert Bus.wait_until(bus, third, 5, '준비', every=0.01) is True and len(polls) == 2                   # ②
assert notes and notes[0].startswith('준비 … 0/5초'), notes                                            # ③

bus, notes, polls = make()
t0 = time.monotonic()
assert Bus.wait_until(bus, lambda: False, 0.3, '못 오는 것', every=0.05) is False                      # ② 시간 초과
assert 0.25 <= time.monotonic() - t0 < 1.0

bus, notes, _ = make()
Bus.wait_until(bus, lambda: False, 0.2, '', every=0.05)
assert notes == []                                                                                     # ④

printed = []
import builtins  # noqa: E402
_print, real = builtins.print, time.monotonic
builtins.print = lambda *a, **k: printed.append(' '.join(map(str, a)))
try:
    bus, _, _ = make(busy=False)
    Bus.wait_until(bus, lambda: False, 0.2, '기다림', every=0.05)
    assert printed == [], printed                                                                      # ⑤ 짧으면 조용
    printed.clear()
    clock = {'t': real()}
    time.monotonic = lambda: clock['t']
    bus, _, _ = make(busy=False)
    bus._poll = lambda s=0.2: clock.__setitem__('t', clock['t'] + 2.0)   # poll 한 번에 2초 흐름
    Bus.wait_until(bus, lambda: False, 12, '오래 걸림')
    time.monotonic = real
    assert len(printed) == 2 and '오래 걸림' in printed[0], printed                                    # 6초 · 10초에 한 줄씩
finally:
    builtins.print = _print
    time.monotonic = real

bus, notes, _ = make()
fut = NS(done=lambda: True)
srv = NS(server_is_ready=lambda: False)
bus.wait_future = lambda f, t, w='': Bus.wait_future(bus, f, t, w)
assert Bus.wait_future(bus, fut, 1.0, '응답') is True                                                  # ⑥
assert Bus.wait_server(bus, srv, '/navigate_to_pose', 0.2) is False
assert any('/navigate_to_pose 서버 연결 대기' in n for n in notes)

src = (Path(__file__).resolve().parent.parent / 'op' / 'bus.py').read_text()
loops = re.findall(r'while [^\n]*time\.monotonic\(\)[^\n]*', src)
assert sorted(loops) == ['while time.monotonic() - start < duration:', 'while time.monotonic() < end and rclpy.ok():'], loops   # ⑦
assert 'wait_for_service' not in src
print('PASS test_wait_until')
