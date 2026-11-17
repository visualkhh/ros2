"""CollisionState — 충돌 감시 칸 (collision_monitor 켬/끔 + 지금 동작). Enter로 켜고 끈다."""
from __future__ import annotations

from ...defs import STAY
from ...terminal import BOLD, DIM, GREEN, RED, YELLOW
from .state import State


class CollisionState(State):
    focusable = True

    def text(self) -> str:
        on = self.state.collision_enabled
        action = self.state.collision_action()
        if on is None and not action:
            return '충돌감시 ' + self.term.paint('-', DIM)
        short = '●' if on is not False else '○'
        color = GREEN if on is not False else YELLOW
        return '충돌감시 ' + self.term.paint(short, color) + (
            ' ' + self.term.paint(action, RED + BOLD) if action else '')

    def activate(self):
        cur = self.state.collision_enabled
        if cur is None:
            cur = self.bus.poll_collision_enabled()
        if cur is None:
            self.ui.modal_key('충돌 감시', [
                '상태를 모름 — /collision_monitor 파라미터 enabled 응답 없음',
                'Nav2(collision_monitor)가 떠 있는지 확인하세요.'])
            return STAY
        want_on = not cur
        apply = '켬' if want_on else '끔 (근접 정지 없이 통과 — 주의)'
        body = [f'현재: {"켬" if cur else "끔"}', f'적용: {apply}', '']
        if not want_on:
            body.insert(2, '끄면 벽·물체 앞에서 자동으로 서지 않습니다. 직접 몰 때만 쓰세요.')
        if not self.ui.confirm('충돌 감시를 켤까요?' if want_on else '충돌 감시를 끌까요?',
                               default=want_on, body=body):
            return STAY
        try:
            ok, text = self.bus.set_collision(want_on)
        except Exception as exc:   # 서비스 응답 형식이 달라도 TUI가 죽지 않게
            ok, text = False, str(exc)
        self.ui.modal_key('충돌 감시', [('적용: ' if ok else '실패: ') + text])
        return STAY
