"""화면 전부 — 4분할 프레임 + 방향키 선택 + 모달 입력.

term=Terminal, state=StateStore(상태바용), procs=Procs(상태바용), poll=ROS 콜백 돌리기.
"""
from __future__ import annotations

import math
import sys
import time

from .dialogs import Dialog, run_select
from .terminal import BOLD, CYAN, DIM, GREEN, RED, RESET, YELLOW, Terminal


class Ui:
    def __init__(self, term: Terminal, state, procs, poll):
        self.term = term
        self.procs = procs
        # 화면 그리기 전용 — capture()가 sys.stdout을 가로채도 UI는 여기로 쓴다
        self.term.out = self.console = sys.stdout
        self.state = state
        self._poll = poll
        self._sel_idx = 0
        self._frame_size = None

    def capture(self):
        """with 블록 안의 print()를 가로챈다. 메뉴 실행 결과를 결과창에 넣기용.
        (Ui 자체 그리기는 self.console이라 영향 없음)"""
        import contextlib
        import io

        @contextlib.contextmanager
        def _cap():
            buf_out, buf_err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(buf_out), contextlib.redirect_stderr(buf_err):
                yield buf_out, buf_err

        return _cap()

    def confirm(self, prompt: str, default: bool = False, body: list[str] | None = None) -> bool:
        """예/아니오 — y 예 · n/Esc/◀ 아니오 · Enter 기본값. 경고·맥락은 body에 (확인 전에 보이게)."""
        suffix = 'Y/n' if default else 'y/N'
        lines = list(body or []) + ([''] if body else []) + [
            prompt, self.term.paint(f'[{suffix}]  y 예 · n/Esc 아니오 · Enter 기본', DIM)]
        key = self.modal_key('확인', lines)
        if key == 'enter':
            return default
        return key == 'y'

    def read_float(self, prompt: str, current: float,
                   positive: bool = False) -> tuple[float, bool]:
        """숫자 한 개 — (값, 확정). Enter 빈칸=현재값 유지 · Esc=(current, False)."""
        def check(raw: str) -> str | None:
            if not raw.strip():
                return None
            try:
                value = float(raw)
            except ValueError:
                return f'숫자가 아님: {raw!r}'
            if not math.isfinite(value):
                return '유한한 숫자가 아님'
            if positive and value <= 0.0:
                return '0보다 커야 합니다'
            if not positive and value < 0:
                return '0 이상이어야 합니다'
            return None

        raw = self.modal_input('값 입력', [prompt, f'현재: {current:g} (빈칸 Enter=유지)'],
                               '값>', validate=check)
        if raw is None:
            return current, False
        return (float(raw) if raw.strip() else current), True

    def reset_selection(self) -> None:
        """메뉴 진입 시 포커스 초기화 (항상 첫 항목)."""
        self._sel_idx = 0

    def _erase_rect(self, left: int, top: int, w: int, h: int) -> None:
        """박스 잔상 지우기 (리사이즈 전 위치)."""
        blank = ' ' * w
        for r in range(top, top + h + 1):
            self.console.write(f'\033[{r};{left}H{blank}')
        self.console.flush()

    def modal_input(self, title: str, body: list[str], prompt: str,
                    default: str = '', validate=None) -> str | None:
        """한 줄 입력 — Enter 확인 · Esc/◀ 취소(None). validate는 Dialog.input 참고."""
        return Dialog(self, title, body).input(prompt, default, validate)

    def busy(self, label: str) -> None:
        """누르자마자 '처리 중' 창 — 기다리지 않음 (원격 SSH·ROS 대기로 몇 초 걸려도 눌린 걸 알게)."""
        import sys
        if not sys.stdout.isatty():
            return
        Dialog(self, '처리 중…', [label, self.term.paint('요청을 보냈습니다. 결과가 나오면 창이 바뀝니다.', DIM)]).draw('key')

    def modal_key(self, title: str, body: list[str]) -> str | None:
        """단일 키 확인 — Enter='enter' · Esc/◀=None · 문자 그대로."""
        return Dialog(self, title, body).key()

    def pager(self, title: str, text: str) -> str:
        """결과 페이저 — ↑↓ 스크롤 · Enter/Esc/◀ 닫기 (메뉴·종료 이동 없음)."""
        return Dialog(self, title, text.splitlines() or ['(출력 없음)']).pager()

    def select(self, title: str, items: list[tuple[str, str]], center: list[str],
                   back: bool = True, main: bool = True,
                   timeout: float | None = None) -> str:
        """메뉴 선택 — 키 규약은 dialogs.py docstring."""
        return run_select(self, title, items, center, back, main, timeout)

    def draw(self, title: str, left: list[str], center: list[str],
                 back: bool = True, main: bool = True, poll: bool = True, focus: int | None = None):
        """4분할 화면: 상단 타이틀+상태바 고정 · 왼쪽 메뉴 · 중앙 내용 · 하단 힌트+입력.

        터미널 크기에 맞춰 다시 계산하므로 창을 늘리면 다음 그리기부터 넓어진다.
        폭이 좁으면(<70) 세로로 쌓는다.
        전체 지우기 없이 커서를 맨 위로 옮겨 제자리에서 다시 그려 깜빡이지 않는다
        (처음·크기 변경 때만 전체 지우기).
        poll=False면 ROS 폴링을 건너뛴다 (방향키 연속 입력 때 0.3초 지연 방지).
        focus: 왼쪽 메뉴에서 고른 줄 — 메뉴가 화면보다 길면 그 줄이 보이게 위아래로 스크롤한다.
        """
        if poll:
            self._poll(0.3)
        cols, rows = self.term.size()
        size = (cols, rows)
        if cols < 70 or rows < 18:
            self.term.clear()
            for line in self.top_lines(title, cols):
                self.console.write(self.term.fit(line, cols) + '\n')
            for line in left:
                self.console.write(self.term.fit(line, cols) + '\n')
            for line in center:
                self.console.write(self.term.fit(line, cols) + '\n')
            self.console.write('-' * cols + '\n')
            self.console.write(self.term.fit(self.hints_line(back=back, main=main), cols) + '\n')
            self.console.flush()
            self._frame_size = size
            return
        if self._frame_size != size:
            self.term.clear()
            self._frame_size = size
        else:
            self.console.write('\033[H')
            self.console.flush()
        top = [self.term.fit(ln, cols) for ln in self.top_lines(title, cols)]
        # 왼쪽(메뉴) 폭은 라벨에 맞춘다 — 최소 24, 최대 화면 절반
        lw = max(24, min(cols // 2, max([self.term.disp_len(l) for l in left] + [0]) + 1))
        cw = cols - lw - 4
        body_h = max(1, rows - 1 - len(top) - 2)
        left_lines = list(left)
        center_lines = list(center)
        if len(left_lines) > body_h:
            left_lines = self._scroll(left_lines, body_h, focus)
        if len(center_lines) > body_h:
            center_lines = center_lines[:body_h - 1] + [f' … 외 {len(center_lines) - body_h + 1}줄']
        frame = list(top)
        for i in range(body_h):
            l = self.term.fit(left_lines[i] if i < len(left_lines) else '', lw)
            c = self.term.fit(center_lines[i] if i < len(center_lines) else '', cw)
            frame.append(f' {l}| {c}')
        frame.append('-' * cols)
        frame.append(self.term.fit(self.hints_line(back=back, main=main), cols))
        # 항상 rows-1줄로 정규화 → 짧아진 프레임의 남은 줄 흔적이 남지 않는다.
        if len(frame) > rows - 1:
            frame = frame[:rows - 1]
        else:
            frame += [''] * (rows - 1 - len(frame))
        for ln in frame:
            self.console.write('\033[K' + ln if ln else '\033[K')
            self.console.write('\n')
        self.console.flush()

    @staticmethod
    def _scroll(lines: list[str], height: int, focus: int | None) -> list[str]:
        """height줄 창으로 자르기 — focus 줄이 창 안에 오게, 잘린 쪽은 '↑ 위 N줄'·'↓ 아래 N줄'로 표시."""
        if height < 3:
            return lines[:height]
        f = 0 if focus is None else max(0, min(focus, len(lines) - 1))
        room = height - 2  # 위·아래 표시 줄 자리
        start = max(0, min(f - room // 2, len(lines) - room))
        view = lines[start:start + room]
        above, below = start, len(lines) - start - len(view)
        return ([f'  ↑ 위 {above}줄' if above else ''] + view +
                [f'  ↓ 아래 {below}줄 (↓로 내리기)' if below else ''])

    def hints_line(self, back: bool = True, main: bool = True) -> str:
        """하단 고정 키 안내 — 모든 메뉴 공통 (최상위면 뒤로·메인 없음)."""
        hints = ['↑↓ 이동', 'Enter 실행', '번호 포커스']
        if back:
            hints.append('Esc/◀ 뒤로')
        if main:
            hints.append('m 메인')
        hints += ['r 새로고침', 'q 종료']
        return '  ' + ' · '.join(hints)

    def top_lines(self, title: str, width: int = 64) -> list[str]:
        """상단 고정 영역: 타이틀 3 + 상태바 2 + 구분선 1."""
        from .procs import SERVICES
        bold = BOLD if self.term.color else ''
        reset = RESET if self.term.color else ''
        lines = [
            f'{bold}{"=" * width}{reset}',
            f'{bold}  Robot Tool  ·  {title}{reset}',
            f'{bold}{"=" * width}{reset}',
        ]
        procs = []
        for name, svc in SERVICES.items():
            on = self.procs.running(name)
            procs.append(self.term.paint(f'{svc.label} {"ON" if on else "off"}',
                                         GREEN + BOLD if on else DIM))
        now_wall = time.strftime('%H:%M:%S', time.localtime())
        lines.append('  ' + ' | '.join(procs) + f' | {self.term.paint(now_wall, DIM)}')
        st = self.state

        def dot(rate, label):
            age = rate.age()
            if age is None:
                return self.term.paint(f'{label} ✗', RED)
            if age > 3.0:
                return self.term.paint(f'{label} {age:.0f}s', YELLOW)
            return self.term.paint(f'{label} {rate.hz():.0f}Hz', GREEN)

        map_txt = dot(st.map, 'map')
        if st.map_info:
            w, h, res = st.map_info
            map_txt += f' {w * res:.1f}x{h * res:.1f}m'
        lines.append(f'  {dot(st.odom, "odom")} | {dot(st.scan, "scan")} | {map_txt}'
                     + (f' | 목표 {st.goal_status}' if st.goal_status else ''))
        # 로봇(OpenCR) 상태: opencr_driver의 /diagnostics — 연결·모터 에러·온도·전류
        level, summary = st.robot_parts()
        color = {0: GREEN, 1: YELLOW, 2: RED + BOLD}.get(level, YELLOW)
        mark = {0: '●', 1: '▲', 2: '✗'}.get(level, '?')
        hint = ' → 상태 메뉴 2) 모터 재부팅' if level == 2 and 'motor error' in st.robot_msg else ''
        lines.append(f'  {self.term.paint(f"로봇 {mark} {summary}{hint}", color)}')
        dom_color = RED + BOLD if st.st_estop else (YELLOW if st.st_control is None or '응답 없음' in st.route_txt() else CYAN)
        lines.append(f'  {self.term.paint(st.domain_txt(), dom_color)}')
        lines.append('-' * width)
        return lines
