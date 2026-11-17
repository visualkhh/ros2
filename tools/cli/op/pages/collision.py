"""CollisionMenu — collision_monitor 설정 (켬/끔 · 정지 거리 시간 · 감시 소스).

Nav2 collision_monitor는 `enabled`(전체)와 `FootprintApproach.time_before_collision`(충돌 몇 초 전에
멈출지), `scan.enabled`(라이다 소스) 등을 파라미터로 갖는다. 일부 polygon 값은 Nav2가 configure 때
읽으므로, 바꾼 뒤 동작이 그대로면 Nav를 재시작해야 할 수 있다.
"""
from __future__ import annotations

from ..defs import STAY
from ..terminal import DIM, YELLOW
from ..menus import ActionItem, Menu


class CollisionMenu(Menu):
    title = '충돌 감시 설정 (collision_monitor)'

    NAMES = (
        'enabled',
        'FootprintApproach.enabled',
        'FootprintApproach.time_before_collision',
        'FootprintApproach.action_type',
        'FootprintApproach.min_points',
        'source_timeout',
        'stop_pub_timeout',
        'scan.enabled',
    )
    ATTR = {
        'enabled': '전체 켬/끔',
        'FootprintApproach.enabled': '로봇 발자국 접근 감시',
        'FootprintApproach.time_before_collision': '충돌 몇 초 전에 멈춤',
        'FootprintApproach.action_type': '동작 종류 (approach/stop/…)',
        'FootprintApproach.min_points': '감지 최소 점 수',
        'source_timeout': '센서 끊김 허용 시간 [s]',
        'stop_pub_timeout': '정지 명령 유지 시간 [s]',
        'scan.enabled': '라이다(/scan) 감시',
    }

    def __init__(self, app):
        super().__init__(app)
        self._vals: dict = {}
        self._at = 0.0

    # ---------------------------------------------------------------- 값 읽기·쓰기
    def _refresh(self) -> None:
        import time
        if time.monotonic() - self._at < 1.0:
            return
        self._at = time.monotonic()
        target = self.bus.param_target('collision_monitor')
        if target is None:
            self._vals = {}
            return
        cli = self.bus.param_client(*target)
        if cli is None:
            return
        got = self.bus.param_get_vals(cli, list(self.NAMES), timeout=0.8)
        if not got:
            return
        self._vals = {n: self.bus.param_display_value(v.type, v) for n, v in zip(self.NAMES, got)}

    def _client(self):
        target = self.bus.param_target('collision_monitor')
        return self.bus.param_client(*target) if target else None

    def _set(self, name: str, ptype: int, value) -> None:
        cli = self._client()
        if cli is None:
            print('collision_monitor 없음 — Nav(4)가 떠 있는지 확인')
            return
        ok, msg = self.bus.param_set(cli, name, ptype, value)
        self._at = 0.0
        if ok:
            self._refresh()
            print(f'{name} = {value}')
        else:
            print(f'설정 실패: {msg}')

    def _bool(self, name: str, label: str, danger: str | None = None) -> None:
        self._refresh()
        cur = self._vals.get(name)
        if cur is None:
            print('현재 값을 못 읽음 — Nav(4)·collision_monitor 확인')
            return
        want = not bool(cur)
        body = [self.ATTR.get(name, name), f'현재: {"켬" if cur else "끔"}',
                f'바꿀 값: {"켬" if want else "끔"}']
        if danger and not want:
            body.insert(2, self.term.paint(danger, YELLOW))
        if not self.ui.confirm(f'{label} — {"켤까요?" if want else "끌까요?"}', default=False, body=body):
            return
        from rcl_interfaces.msg import ParameterType as PT
        self._set(name, PT.PARAMETER_BOOL, want)

    def _toggle_enabled(self) -> None:
        cur = self.state.collision_enabled
        if cur is None:
            cur = self.bus.poll_collision_enabled()
        if cur is None:
            print('enabled 상태를 못 읽음 — Nav(4)·collision_monitor 확인')
            return
        want = not cur
        if not self.ui.confirm(f'충돌 감시를 {"켤까요?" if want else "끌까요?"}', default=False,
                               body=[f'현재: {"켬" if cur else "끔"}',
                                     '끄면 벽·물체 앞에서 자동으로 서지 않습니다.']):
            return
        ok, text = self.bus.set_collision(want)
        print(('적용: ' if ok else '실패: ') + text)

    def _set_float(self, name: str, label: str, maximum: float | None = None) -> None:
        self._refresh()
        cur = self._vals.get(name)
        if cur is None:
            print('현재 값을 못 읽음 — Nav(4)·collision_monitor 확인')
            return
        value, ok = self.ui.read_float(f'{label} [{name}]', float(cur),
                                       positive=True, maximum=maximum)
        if not ok:
            return
        from rcl_interfaces.msg import ParameterType as PT
        self._set(name, PT.PARAMETER_DOUBLE, float(value))

    # ---------------------------------------------------------------- 화면
    def items(self):
        self._refresh()
        v = self._vals
        enabled = self.state.collision_enabled
        ap = v.get('FootprintApproach.enabled')
        scan = v.get('scan.enabled')
        return [
            ActionItem(self.app, '1', lambda: f'전체 {"끄기" if enabled else "켜기"} (지금 {"켬" if enabled else "끔" if enabled is False else "? "})',
                       self._toggle_enabled),
            ActionItem(self.app, '2',
                       lambda: f'충돌 전 정지 시간 (time_before_collision) = {v.get("FootprintApproach.time_before_collision", "?")} s',
                       lambda: self._set_float('FootprintApproach.time_before_collision',
                                               '충돌 전 정지 시간', 10.0)),
            ActionItem(self.app, '3',
                       lambda: f'발자국 접근 감시 (FootprintApproach.enabled) = {"켬" if ap else "끔" if ap is False else "?"}',
                       lambda: self._bool('FootprintApproach.enabled', '발자국 접근 감시')),
            ActionItem(self.app, '4',
                       lambda: f'라이다 감시 (scan.enabled) = {"켬" if scan else "끔" if scan is False else "?"}',
                       lambda: self._bool('scan.enabled', '라이다 감시',
                                          danger='끄면 라이다로 장애물을 못 봅니다 — 직접 조작할 때만!')),
            ActionItem(self.app, '5',
                       lambda: f'센서 끊김 허용 (source_timeout) = {v.get("source_timeout", "?")} s',
                       lambda: self._set_float('source_timeout', '센서 끊김 허용', 10.0)),
            ActionItem(self.app, '6',
                       lambda: f'정지 명령 유지 (stop_pub_timeout) = {v.get("stop_pub_timeout", "?")} s',
                       lambda: self._set_float('stop_pub_timeout', '정지 명령 유지', 10.0)),
            ActionItem(self.app, '7', '값 다시 읽기', self._force_refresh),
        ]

    def _force_refresh(self) -> None:
        self._at = 0.0
        self._refresh()

    def center(self):
        self._refresh()
        v = self._vals
        on = self.state.collision_enabled
        action = self.state.collision_action()
        lines = ['[collision_monitor]',
                 f' 전체: {"● 켬" if on else "○ 끔" if on is False else "- 모름"}'
                 + (f' · 지금 동작: {action}' if action else '')]
        if not v:
            lines.append(self.term.paint(' 파라미터 응답 없음 — Nav(4)와 collision_monitor가 떠 있는지 확인', YELLOW))
        for name in self.NAMES:
            lines.append(f' {name} = {v.get(name, "?")}')
        lines += ['', self.term.paint(
            ' 1 전체 켬/끔 · 2 충돌 전 정지 시간 · 3 발자국 감시 · 4 라이다 감시 · 5 source_timeout · 6 stop_pub_timeout · 7 새로고침', DIM),
                  self.term.paint(
            ' 일부 polygon 값은 Nav2 configure 때만 읽힐 수 있음 — 바꾼 뒤 동작이 그대로면 Nav 재시작', DIM)]
        return lines


__all__ = ['CollisionMenu']
