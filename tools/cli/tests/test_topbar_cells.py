#!/usr/bin/env python3
"""상단 상태바 조작·위치추정·지도 칸 시험 — ROS 없이: python3 tools/cli/tests/test_topbar_cells.py

확인: ① 세 칸 모두 Tab 포커스 가능 (조작 · 위치추정 · 지도 순) ② Enter = 각각 'mode' · 'localization' · 'map-change' 화면
③ 표시: 조작 NAV · 위치추정 LOCALIZATION/MAPPING · 지도 이름 (예전엔 '지도: LOCALIZATION track' 한 칸이었음)
④ 화면 이름이 operator_tool.screen 에 등록됨 · 화면(위치추정 메뉴·지도 바꾸기)이 열림
"""
import re
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
import geometry_msgs.msg as _gm  # noqa: E402
for _n in ('PoseStamped', 'PoseWithCovarianceStamped', 'Twist', 'TwistStamped'):   # 가짜 ROS에 없는 메시지 — import 만 되게
    if not hasattr(_gm, _n):
        setattr(_gm, _n, type(_n, (), {}))
from op.bars.state.map_state import LocalizationState, MapState  # noqa: E402
from op.bars.state.operation_state import OperationState  # noqa: E402
from op.bars.state.top_state_bar import TopStateBar  # noqa: E402

strip = lambda t: re.sub(r'\x1b\[[0-9;]*m', '', t)   # noqa: E731
paint = lambda t, c: t   # noqa: E731


def cell(cls, **st):
    went = []
    ui = NS(term=NS(paint=paint), state=NS(**st), window=NS(navigate=lambda name, **kw: (went.append(name), 'back')[1]))
    return cls(ui), went


order = [type(s).__name__ for s in TopStateBar(MagicMock()).focus_states()]
assert order[:3] == ['OperationState', 'LocalizationState', 'MapState'], order                       # ①

for cls, screen in ((OperationState, 'mode'), (LocalizationState, 'localization'), (MapState, 'map-change')):
    c, went = cell(cls, st_control=None, st_operation=None, st_map=None)
    assert c.focusable and c.activate() == 'back' and went == [screen], (cls, went)                  # ②

assert strip(cell(OperationState, st_control='NAV_PLAN')[0].text()) == '조작 NAV'                    # ③
assert strip(cell(LocalizationState, st_operation='LOCALIZATION')[0].text()) == '위치추정: LOCALIZATION'
assert strip(cell(LocalizationState, st_operation='SLAM')[0].text()) == '위치추정: MAPPING'
assert strip(cell(LocalizationState, st_operation=None)[0].text()) == '위치추정: -'
assert strip(cell(MapState, st_map='track')[0].text()) == '지도: track'
assert strip(cell(MapState, st_map='(SLAM 작성 중)')[0].text()) == '지도: SLAM'
assert strip(cell(MapState, st_map='')[0].text()) == '지도: -'
assert strip(cell(MapState, st_map='a' * 30)[0].text()) == '지도: ' + 'a' * 19 + '…'

src = (Path(__file__).resolve().parent.parent / 'operator_tool.py').read_text()                       # ④
for name, cls in (('mode', 'ModeMenu'), ('localization', 'LocalizationMenu'), ('map-change', 'MapChangePage')):
    assert f"'{name}': pages.{cls}" in src, name
from op.pages.services import LocalizationMenu, MapChangePage  # noqa: E402
app = NS(state=NS(st_operation='LOCALIZATION', st_map='track'), bus=NS(), ui=NS(), term=NS(paint=paint))
menu = LocalizationMenu(app)
assert [i.key for i in menu.items()] == ['1', '2']
text = '\n'.join(menu.center())
assert 'Nav (LOCALIZATION)' in text and 'track' in text
app.state.st_operation = 'SLAM'
assert 'SLAM (MAPPING)' in '\n'.join(menu.center())
assert MapChangePage(app).title == '지도 바꾸기'
print('PASS test_topbar_cells')
