#!/usr/bin/env python3
"""cli-test 시나리오 — 실제 pty 키 입력으로 UI 흐름을 검증한다 (가짜 ROS층, ROS 불필요).

실행: python3 tool/cli-test/scenarios.py [시나리오 이름 ...]
"""
from __future__ import annotations

import os
import re
import select
import signal
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CLI = HERE.parent / 'cli'

UP, DOWN, LEFT, RIGHT = b'\x1b[A', b'\x1b[B', b'\x1b[D', b'\x1b[C'
ENTER, ESC, BACKSPACE = b'\r', b'\x1b', b'\x7f'


def plain(data: bytes) -> str:
    text = data.decode('utf-8', errors='replace')
    return re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', text)


class Session:
    def __init__(self, name: str):
        self.name = name
        self.out = bytearray()
        self.pid, self.fd = os.forkpty() if hasattr(os, 'forkpty') else (None, None)
        if self.pid == 0:
            os.chdir(HERE.parent.parent)
            os.execv(sys.executable, [sys.executable, str(HERE / 'cli_test.py'), 'menu'])
        if self.pid is None:
            import pty
            self.pid, self.fd = pty.fork()
            if self.pid == 0:
                os.chdir(HERE.parent.parent)
                os.execv(sys.executable, [sys.executable, str(HERE / 'cli_test.py'), 'menu'])
        self.closed = False
        self.mark = 0
        self.expect('메인 메뉴')  # 첫 화면이 뜬 뒤부터 키를 보낸다

    def _read(self, timeout=0.2) -> bool:
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return False
        try:
            data = os.read(self.fd, 65536)
        except OSError:
            self.closed = True
            return False
        if not data:
            self.closed = True
            return False
        self.out.extend(data)
        return True

    def send(self, data: bytes) -> None:
        while self._read(0.05):  # 앞 화면 출력은 다 받아 두고
            pass
        self.mark = len(self.out)  # expect/absent는 이 키를 보낸 뒤 출력만 본다
        os.write(self.fd, data)

    def expect(self, marker: str, timeout=8.0) -> None:
        target = marker.encode('utf-8')
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if target in self.out[self.mark:]:
                return
            self._read(max(0.0, min(0.2, end - time.monotonic())))
        raise AssertionError(f'[{self.name}] 기대 문자열 없음: {marker!r}')

    def absent(self, marker: str, wait=0.8) -> None:
        end = time.monotonic() + wait
        while time.monotonic() < end:
            self._read(0.1)
        if marker.encode('utf-8') in self.out[self.mark:]:
            raise AssertionError(f'[{self.name}] 나오면 안 되는 문자열: {marker!r}')

    def step(self, data: bytes, marker: str | None = None, wait=0.7) -> None:
        self.send(data)
        if marker is not None:
            self.expect(marker)
        else:
            end = time.monotonic() + wait  # 기다리는 동안도 읽음 — 안 읽으면 pty가 차서 CLI가 출력에서 멈추고,
            while time.monotonic() < end:  #   뒤에 보낸 키가 ESC와 붙어 한 시퀀스로 먹힘 (상태바 1초 갱신 뒤 manual-cancel 실패)
                self._read(0.05)
            while self._read(0.05):
                pass

    def finish(self, marker='robot_tool 종료', timeout=8.0) -> None:
        self.step(b'q', marker, wait=0) if not self.closed else None
        if not self.closed:
            self.expect(marker, timeout)

    def close(self) -> None:
        if self.closed:
            return
        try:
            os.kill(self.pid, signal.SIGKILL)
        except OSError:
            pass
        self.closed = True
        try:
            os.close(self.fd)
        except OSError:
            pass
        try:
            os.waitpid(self.pid, 0)
        except ChildProcessError:
            pass

    def assert_clean(self) -> None:
        text = plain(bytes(self.out))
        if 'Traceback' in text:
            raise AssertionError(f'[{self.name}] Traceback:\n{text[-2000:]}')


SCENARIOS = []


def scenario(name: str):
    def deco(fn):
        def wrapped():
            s = Session(name)
            try:
                fn(s)
                s.assert_clean()
                print(f'PASS {name}')
            except Exception:
                (HERE / f'.scenario-{name}.bin').write_bytes(bytes(s.out))
                raise
            finally:
                s.close()
        wrapped.__name__ = name
        SCENARIOS.append(wrapped)
        return wrapped
    return deco


@scenario('menu-arrows-esc')
def _(s):
    # 알려진 현상: 시작 후 첫 ↓ 하나는 먹히지 않음 (변경 전 코드도 같음) — 그래서 ↓ 두 번 = 2번째 항목
    s.step(DOWN, wait=0.5)
    s.step(DOWN, wait=0.5)
    s.step(ENTER, '·  베이스')              # 2) 베이스
    s.step(ESC, '메인 메뉴')
    s.finish()


@scenario('nested-back-one-level')
def _(s):
    s.step(b'8', wait=0.3)                    # 노드
    s.step(ENTER, '·  노드')
    s.step(ESC, '·  메인 메뉴')
    s.step(ESC, '·  메인 메뉴')              # 최상위 Esc는 무시
    s.finish()


@scenario('teleop-confirm-exit')
def _(s):
    s.step(b'5', wait=0.3)
    s.step(ENTER, '수동 주행')
    s.step(b'1', wait=0.2)
    s.step(ENTER, 'MANUAL로 바꿀까요?')      # 텔레오퍼 — 확인창이 먼저
    s.step(b'y', '텔레오퍼 →')
    s.step(b'w', 'v=+0.15')                   # w = 전진 명령
    s.step(ESC, '텔레오퍼 종료')               # Esc → 창이 닫히고 결과창(출력 모음)
    s.step(ENTER, '·  수동 주행')              # 결과창 닫으면 같은 메뉴
    s.finish()


@scenario('ab-route-typed-undo')
def _(s):
    route = Path(__file__).resolve().parents[1] / 'bags' / 'ab' / 'route.json'
    if route.exists():
        print('SKIP ab-route-typed-undo: 지정한 route.json이 이미 있음 — 지우지 않으려고 건너뜀')
        return
    try:
        for _ in range(9):                    # b) A/B (첫 키 하나는 시작 직후라 먹힐 수 있어 한 번 더)
            s.step(DOWN, wait=0.3)
        s.step(ENTER, 'A/B 테스트 (노드별)')
        s.step(ENTER, 'A/B 테스트 · rl_nav_controller')
        s.step(b'1', wait=0.2)
        s.step(ENTER, 'A/B 경로 편집')
        s.step(b'5', wait=0.2)
        s.step(ENTER, 'goal 추가')
        s.step(b'1.5 0.5 90' + ENTER, '저장:')   # 좌표 입력 → route.json 저장 (결과창)
        s.step(ENTER, 'G1')
        assert route.is_file() and '1.5' in route.read_text(), 'route.json 저장 안 됨'
        s.step(b'6', wait=0.2)
        s.step(ENTER, 'route.json 삭제')           # 마지막 goal 삭제 → 파일 삭제
        s.step(ENTER, 'A/B 경로 편집')
        assert not route.exists(), 'route.json이 안 지워짐'
    finally:
        route.unlink(missing_ok=True)


@scenario('ab-route-map-pick')
def _(s):
    route = Path(__file__).resolve().parents[1] / 'bags' / 'ab' / 'route.json'
    if route.exists():
        print('SKIP ab-route-map-pick: 지정한 route.json이 이미 있음 — 지우지 않으려고 건너뜀')
        return
    try:
        for _ in range(9):                    # b) A/B
            s.step(DOWN, wait=0.3)
        s.step(ENTER, 'A/B 테스트 (노드별)')
        s.step(ENTER, 'A/B 테스트 · rl_nav_controller')
        s.step(b'1', wait=0.2)
        s.step(ENTER, 'A/B 경로 편집')
        s.step(b'1', wait=0.2)                # 1) 지도에서 찍기
        s.send(ENTER)
        s.expect('A/B 경로 — 지도에서 찍기', timeout=20)     # 현재 위치 TF(최대 5초) · /map을 받은 뒤에 열림
        s.step(b'+', '한 칸 =')                              # 지도가 포커스를 받아 바로 확대 (중앙 = 지도)
        s.step(ENTER, 'G1 추가')                             # 십자선 위치(지도 가운데)에 goal
        s.step(RIGHT * 2, '자유이동')                       # 방향키 = 지도를 밂 → 로봇 따라가기가 풀림
        s.step(b']', wait=0.2)                               # 방향 +15°
        s.step(ENTER, 'G2 추가')
        s.step(b'-', wait=0.3)                               # 축소
        s.step(b'r', '따라가기')                         # 다시 로봇 중심
        s.step(ESC, 'Tab 중앙')                              # Esc → 왼쪽 메뉴로 (힌트에 'Tab 중앙')
        s.step(b'9', wait=0.3)
        s.step(ENTER, '저장:')                               # 9) 저장하고 나가기 → route.json 저장
        import json
        goals = json.loads(route.read_text())['route']
        assert len(goals) == 2 and goals[1]['yaw_deg'] == 15.0, goals
    finally:
        route.unlink(missing_ok=True)


@scenario('map-pose-pick')
def _(s):
    s.step(b'6', wait=0.3)                    # 6) 지도/위치
    s.step(ENTER, '·  지도/위치')
    s.step(b'p', wait=0.2)                    # p) 초기 위치 지정 (지도에서 찍기)
    s.send(ENTER)
    s.expect('로봇 위치 지정', timeout=20)    # 지도 화면 (mode=pose) — 열리는 동안 busy 창
    s.step(b']', '방향 +15°')                 # 방향 +15° (지도 포커스의 키)
    s.step(ENTER, '로봇 위치로 지정할까요?')   # 십자선 위치 → 확인창
    s.step(b'y', '위치 지정')                 # /initialpose 발행 (가짜 ROS) → 화면 아래 메시지
    s.step(ESC, 'Tab 중앙')                   # Esc → 왼쪽 메뉴로 (힌트에 'Tab 중앙') · 한 번 더 Esc → 지도/위치 메뉴로
    s.step(ESC, '·  지도/위치')
    s.finish()


@scenario('input-validate')
def _(s):
    s.step(b'5', wait=0.3)
    s.step(ENTER, '수동 주행')
    s.step(b'6', wait=0.3)                    # 선속도 설정
    s.step(ENTER, '값 입력')
    s.step(b'-1' + ENTER, '0보다 커야 합니다')
    s.step(BACKSPACE * 2 + b'0.1' + ENTER, '선속도 = 0.1 m/s')
    s.finish()


def main() -> int:
    only = sys.argv[1:]
    run = [fn for fn in SCENARIOS if not only or fn.__name__ in only]
    failed = []
    for fn in run:
        try:
            fn()
        except Exception as e:
            failed.append((fn.__name__, str(e)))
            print(f'FAIL {fn.__name__}: {e}')
    print(f'\n{len(run) - len(failed)}/{len(run)} 통과')
    return 1 if failed or not run else 0


if __name__ == '__main__':
    raise SystemExit(main())
