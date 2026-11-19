#!/usr/bin/env python3
"""컨트롤러별 파라미터 블록 시험 — ROS 없이: python3 tools/cli/tests/test_fallback_params.py

구조(2026-11-19): nav2_params.yaml 의 FollowPath 아래에 inner_mppi · inner_dwb · inner_rotation_shim 블록이 모두 있고,
플러그인(rl_nav_controller.cpp)이 fallback_controller 가 고른 것의 블록만 읽는다. CLI 는 fallback_controller 한 줄만 바꾼다.
예전 문제: 프리셋 파일을 inner: 에 복사하는 방식이라 프리셋 값(정수 254)이 틀리면 Nav2 시작이 중단됐다.
확인: ① 블록이 FALLBACKS 만큼 다 있다 ② set_fallback 은 그 한 줄만 바꾼다 ③ 없는 이름·블록 없음은 거부 ④ C++ 이름 공간이 YAML 블록과 같다
⑤ 실수여야 하는 값이 정수가 아니다 (rotation_shim max_cost_threshold)
"""
import re
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.procs import Procs  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
PARAMS = ROOT / 'ros2_ws/src/tb3_bringup/config/nav2_params.yaml'
CPP = ROOT / 'ros2_ws/src/rl_nav_controller/src/rl_nav_controller.cpp'
follow = lambda text: yaml.safe_load(text)['controller_server']['ros__parameters']['FollowPath']   # noqa: E731

text = PARAMS.read_text()
f = follow(text)
assert all(f'inner_{n}' in f and isinstance(f[f'inner_{n}'], dict) for n in Procs.FALLBACKS)          # ①
assert 'inner' not in f and f['fallback_controller'] in Procs.FALLBACKS
assert isinstance(f['inner_rotation_shim']['max_cost_threshold'], float)                                # ⑤


def switch(name, src=text):
    tmp = Path(tempfile.mkdtemp()) / 'nav2_params.yaml'
    tmp.write_text(src)
    ok, msg = Procs.set_fallback(NS(FALLBACKS=Procs.FALLBACKS, nav_params_source=lambda: tmp, running=lambda n: False), name)
    return ok, msg, tmp.read_text()


for name in Procs.FALLBACKS:
    ok, msg, new = switch(name)
    assert ok and follow(new)['fallback_controller'] == name, msg
    changed = [(a, b) for a, b in zip(text.splitlines(), new.splitlines()) if a != b]
    assert len(new.splitlines()) == len(text.splitlines()) and len(changed) <= 1                       # ② 그 한 줄만
    assert {k: v for k, v in follow(new).items() if k != 'fallback_controller'} == \
        {k: v for k, v in f.items() if k != 'fallback_controller'}

assert switch('nope')[0] is False                                                                      # ③
assert switch('mppi', text.replace('      inner_mppi:\n', '      inner_x:\n'))[0] is False

cpp = CPP.read_text()
ns = re.search(r'kInnerNs\[\] = \{([^}]*)\}', cpp).group(1)
names = [n.strip().strip('"') for n in ns.split(',')]
assert names == ['.inner', '.inner_dwb', '.inner_mppi', '.inner_rpp', '.inner_rotation_shim'], names  # ④ 인덱스 = inner_kind_
assert all(n.lstrip('.') in f for n in ('inner_dwb', 'inner_mppi', 'inner_rotation_shim'))
print('PASS test_fallback_params')
