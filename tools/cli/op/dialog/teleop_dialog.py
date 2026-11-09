"""TeleopDialog — 텔레오퍼 창 (Dialog 하위 클래스). bus.cmd_teleop이 띄운다."""
from __future__ import annotations

import time

from std_msgs.msg import Empty

from ..terminal import CYAN, DIM
from .dialog import CONTINUE, Dialog

# 수동 주행 속도 상한 (텔레오퍼 e · 메뉴 속도 설정) — Waffle Pi 사양 = 펌웨어 바퀴 상한 8 rad/s × 0.033 m
MAX_LIN, MAX_ANG = 0.26, 1.82
# 텔레오퍼 키 → (전후, 회전) 방향. 사선(둘 다 0 아님) = 설정 선속도 그대로 + 반경 DIAG_RADIUS 원호
#   (ω = v / R → 속도를 바꿔도 휘는 모양이 같음). 0.20 m/s까지 속도 그대로, 그 위는 바깥 바퀴가 상한이라
#   opencr_driver가 두 바퀴를 같은 비율로 줄임 (모양 유지).
# 후진 사선은 회전 부호 반대 (teleop_twist_keyboard와 같음: z = 뒤로 가며 왼쪽으로 휘어짐)
TELEOP_MOVES = {'w': (1, 0), 'x': (-1, 0), 'a': (0, 1), 'd': (0, -1), 's': (0, 0),
                'q': (1, 1), 'e': (1, -1), 'z': (-1, -1), 'c': (-1, 1)}
DIAG_RADIUS = 0.5  # [m] 사선 원호 반경
TELEOP_ALIAS = {'up': 'w', 'down': 'x', 'left': 'a', 'right': 'd', ' ': 's'}  # 방향키·스페이스 = 같은 이동


class TeleopDialog(Dialog):
    """텔레오퍼 창 — w/x/a/d·방향키 이동 · q/e/z/c 사선(원호) · s/space 정지 · -/= 속도± · b 삑 · Esc 종료.
    속도는 방향 × 설정값이라 달리는 중에 바꿔도 바로 반영. 0.1초마다 다시 보냄. 닫을 때 정지는 bus.cmd_teleop이 보낸다.
    on_mark가 있으면 g/Enter에 로봇을 세우고 on_mark() — 반환 문자열을 창 맨 아래에 (A/B 경로 goal 찍기)."""

    def __init__(self, bus, title: str, linear: float, angular: float, on_mark=None):
        super().__init__(bus.ui, title)
        self.bus = bus
        self.move = (0, 0)  # (전후, 회전) 방향
        self.lin_speed, self.ang_speed = min(linear, MAX_LIN), min(angular, MAX_ANG)
        self.last_send = 0.0
        self.last_key, self.key_count = '-', 0
        self.on_mark = on_mark
        self.mark_txt = ''

    def _command(self) -> tuple[float, float]:
        linear = self.move[0] * self.lin_speed
        angular = self.move[1] * (self.lin_speed / DIAG_RADIUS if self.move[0] else self.ang_speed)
        return linear, angular

    def content(self) -> list[str]:
        bus, st, term = self.bus, self.bus.state, self.bus.term
        linear, angular = self._command()
        key_txt = f'키 {self.last_key} ×{self.key_count}' if self.key_count else '키 입력 대기 중'
        lines = [self.title,
                f'명령 v={linear:+.2f} m/s ω={angular:+.2f} rad/s',
                f'odom {st.odom_txt()}',
                bus.wheel_summary(),
                f'scan {st.scan_txt()}',
                f'map  {st.map_txt()}',
                f'{key_txt} · 설정 v {self.lin_speed:.2f} m/s · ω {self.ang_speed:.2f} rad/s',
                term.paint('w/x/a/d·방향키 이동 · q/e/z/c 사선 · s/space 정지', DIM),
                term.paint('-/= 속도± · b 삑 · Esc 종료', DIM)]
        if self.on_mark is not None:
            lines += [term.paint('g/Enter = 지금 위치를 goal로 추가', CYAN)] + ([self.mark_txt] if self.mark_txt else [])
        return lines

    def on_tick(self) -> None:
        now = time.monotonic()
        if now - self.last_send >= 0.1:
            self.bus.send_velocity(*self._command())
            self.last_send = now
        self.ui.tick(0.01, self._geom)   # ROS 폴링 + 창 뒤 상태바·바퀴 표(가리지 않은 줄) 갱신

    def on_key(self, k):
        if k in ('esc', 'eof', '\x03'):
            return None
        if self.on_mark is not None and k in ('g', 'G', 'enter'):
            self.move = (0, 0)  # 세우고 묻는다 — 확인창 동안 굴러가지 않게
            for _ in range(3):
                self.bus.send_velocity(0.0, 0.0)
                self.bus._poll(0.05)
            self.mark_txt = self.on_mark() or ''
            return CONTINUE
        k = TELEOP_ALIAS.get(k, k)
        self.key_count += 1
        self.last_key = 'space' if k == 's' else k
        if k in TELEOP_MOVES:
            self.move = TELEOP_MOVES[k]
        elif k == 'b':
            self.bus._beep_pub.publish(Empty())
        elif k in ('=', '+'):   # 상한 = Waffle Pi 사양
            self.lin_speed, self.ang_speed = min(self.lin_speed + 0.05, MAX_LIN), min(self.ang_speed + 0.3, MAX_ANG)
        elif k in ('-', '_'):
            self.lin_speed, self.ang_speed = max(self.lin_speed - 0.05, 0.05), max(self.ang_speed - 0.3, 0.2)
        return CONTINUE
