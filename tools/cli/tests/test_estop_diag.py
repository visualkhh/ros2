#!/usr/bin/env python3
"""상단 E-STOP · 진단 칸 시험 — ROS 없이: python3 tools/cli/tests/test_estop_diag.py

확인: ① 두 칸 모두 Tab 포커스 가능 ② E-STOP: 현재→적용 다이얼로그, y면 /cmd_manual_estop 요청(켜기 기본 예 · 해제 기본 아니오),
아니오·상태 모름이면 요청 없음 ③ 진단 칸 Enter = 'diagnostics' 화면 ④ 진단 상세(레벨·메시지·값) ⑤ 'diagnostics' 화면이 등록됨
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
from op.bars.state.diag_state import DiagState  # noqa: E402
from op.bars.state.estop_state import EstopState  # noqa: E402
from op.bus import Bus  # noqa: E402
from op.defs import STAY  # noqa: E402

assert EstopState.focusable and DiagState.focusable


def toggle(estop, answer):
    """Bus.toggle_estop 을 가짜 ui·state 로 — (물어본 것, 보낸 요청, 알림)."""
    asked, sent, notes = [], [], []
    bus = NS(state=NS(st_estop=estop),
             ui=NS(confirm=lambda prompt, default=False, body=None: (asked.append((prompt, default)), answer)[1],
                   modal_key=lambda title, lines: notes.append(title)),
             cmd_estop=lambda a: sent.append(a.value))
    Bus.toggle_estop(bus)
    return asked, sent, notes


assert toggle(False, True) == ([('비상정지를 켤까요?', True)], ['on'], [])      # 켜기: 기본 예
assert toggle(True, True) == ([('비상정지를 해제할까요?', False)], ['off'], [])   # 해제: 기본 아니오
assert toggle(False, False)[1] == [] and toggle(True, False)[1] == []         # 아니오 → 요청 없음
assert toggle(None, True) == ([], [], ['비상정지'])                            # 모름 → 묻지도 보내지도 않음

cell = EstopState(NS(term=NS(), state=NS(), bus=NS(toggle_estop=lambda: None)))
assert cell.activate() == STAY

went = []
ui = NS(term=NS(), state=NS(), window=NS(navigate=lambda name: (went.append(name), 'back')[1]))
assert DiagState(ui).activate() == 'back' and went == ['diagnostics']

paint = lambda t, c: t   # noqa: E731
bus = NS(diag_short=Bus.diag_short, term=NS(paint=paint, disp_len=len),
         state=NS(diagnostics={'/a/b': (2, '모터 에러')}, diag_values={'/a/b': [('temp', '71'), ('current', '0.4')]}))
lines = Bus.diag_detail_lines(bus, '/a/b')
assert lines[0] == '[a / b]' and 'ERROR' in lines[1] and '모터 에러' in lines[2] and any('temp' in l and '71' in l for l in lines)
assert 'STALE' in Bus.diag_detail_lines(NS(**{**bus.__dict__, 'state': NS(diagnostics={'x': (3, '끊김')}, diag_values={})}), 'x')[1]
assert 'DiagnosticsMenu' in (Path(__file__).resolve().parent.parent / 'operator_tool.py').read_text()
print('PASS test_estop_diag')
