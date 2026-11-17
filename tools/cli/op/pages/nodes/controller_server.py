"""controller_server — Nav2 FollowPath(rl_nav_controller::RlNavController)의 RL/MPPI 상태.

플러그인이 매 제어 주기 `/controller_server/FollowPath/status`를 발행한다.
  [rl_active, reason, policy_loaded, enable, scan_ok, scan_age, scan_min, v, w, inner_kind]
  reason: 0=RL, 1=모델 없음/로드 실패, 2=enable=false, 3=/scan 없음, 4=추론 실패
  inner_kind: 1=DWB, 2=MPPI, 3=RPP, 4=RotationShim
"""
from __future__ import annotations

import json
import time

from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from ...defs import STAY
from ...terminal import DIM
from ...menus import ActionItem, GoMenu, InfoItem, Menu, MenuItem
from .feature import NodeFeature

LIVE = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
REASONS = {
    0: '',
    1: '모델 없음/로드 실패',
    2: 'enable=false',
    3: '/scan 없음',
    4: '추론 실패',
}
INNER = {1: 'DWB', 2: 'MPPI', 3: 'RPP', 4: 'RotationShim'}

SERVICE_HINTS = {
    '/controller_server/FollowPath/toggle': {
        'label': 'FollowPath 토글 (RL↔fallback)',
        'default': '{"data": true}',
    },
    '/controller_server/set_parameters': {
        'label': '파라미터 설정 (SetParameters)',
        'default': '{"parameters": [{"name": "FollowPath.enable", "value": {"type": 1, "bool_value": false}}]}',
    },
    '/controller_server/get_state': {
        'label': '수명주기 get_state',
        'default': '{}',
    },
}


def _parse_request(text: str) -> dict:
    text = (text or '').strip()
    if not text:
        return {}
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError('JSON 객체({...}) 형태여야 합니다')
    return value


def _validate_request(text: str):
    try:
        _parse_request(text)
        return None
    except ValueError as exc:
        return f'JSON 오류: {exc}'


class ServiceCallItem(MenuItem):
    """서비스 하나 — Enter로 Request JSON 입력 후 호출."""

    def __init__(self, app, name: str, type_name: str, callable_: bool = True,
                 label: str = '', default: str = '{}'):
        super().__init__(app)
        self._name = name
        self._type = type_name
        self._callable = callable_
        self._label = label
        self._default = default

    def label(self):
        tail = '' if self._callable else '  (docker 미지원)'
        shown = self._label or self._name
        return f'{shown}  [{self._type or "타입 ?"}]{tail}'

    def run(self):
        if not self._type:
            print('서비스 타입 정보 없음 — discovery를 기다린 뒤 새로고침하세요')
            return STAY
        body = [f'서비스: {self._name}', f'타입: {self._type}']
        if self._type.endswith('/Trigger') or self._type.endswith('/Empty'):
            body.append('요청 필드 없음')
            request = {}
        else:
            if self._type.endswith('/SetBool'):
                body.append('요청: {"data": true} 또는 {"data": false}')
                default = '{"data": true}'
            else:
                body.append('Request 메시지 필드 JSON — 빈 값 = {}')
                default = self._default or '{}'
            text = self.ui.modal_input('서비스 호출', body, 'request>',
                                       default=default, validate=_validate_request)
            if text is None:
                return STAY
            try:
                request = _parse_request(text)
            except ValueError as exc:
                print(f'JSON 오류: {exc}')
                return STAY
        body += ['', '요청: ' + json.dumps(request, ensure_ascii=False)]
        if not self._callable:
            body.append('도커 service_action_bag_input 미지원 — 실패할 수 있음')
        if not self.ui.confirm(f'{self._name} 호출할까요?', default=False, body=body):
            return STAY
        ok, result = self.bus.call_service(self._name, self._type, request)
        if ok:
            print(f'[{self._name}] 응답')
            print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
        else:
            print(f'[{self._name}] 실패: {result}')
        return STAY


class ServiceMenu(Menu):
    """노드 서비스 목록 — Enter로 호출."""

    def __init__(self, app, node_name: str, entries: list[dict] | None = None):
        super().__init__(app)
        self.node_name = node_name
        self.title = f'서비스 · {node_name}'
        if entries is None:
            entries = [{'name': n, 'type': t, 'callable': c, 'label': '', 'default': '{}'}
                       for n, t, c in self.bus.node_services(node_name)]
        self._entries = entries

    def items(self):
        if not self._entries:
            return [InfoItem(self.app, '서비스 없음 (노드가 안 떠 있거나 discovery 대기)')]
        return [ServiceCallItem(self.app, e['name'], e.get('type', ''), e.get('callable', True),
                                e.get('label', ''), e.get('default', '{}'))
                for e in self._entries]

    def center(self):
        lines = [f'[{self.node_name}]', f' 서비스 {len(self._entries)}개', '',
                 ' Enter: Request 입력 → 호출 · Esc/◀ 뒤로']
        lines.append(self.term.paint(' 도커: service_action_bag_input이 타입이 고정된 서비스만 실제 호출', DIM))
        return lines + self.ui.center_summary()


class ControllerServerFeature(NodeFeature):
    node = 'controller_server'

    def wire(self, tool) -> None:
        tool.create_subscription(
            Float32MultiArray, '/controller_server/FollowPath/status',
            lambda m: tool.state.on_rl_status(m.data), LIVE)
        # status가 없는 구버전에서도 최소한 주행 여부는 볼 수 있게
        tool.create_subscription(
            TwistStamped, '/cmd_vel_nav',
            lambda _m: tool.state.cmd_vel_nav.tick(), LIVE)

    def services(self, bus, name: str) -> list[dict]:
        """controller_server가 제공하는 서비스 카탈로그 — discovery + 전용 힌트."""
        rows = []
        for svc_name, type_name, callable_ in bus.node_services(name):
            hint = SERVICE_HINTS.get(svc_name, {})
            rows.append({
                'name': svc_name,
                'type': type_name,
                'callable': callable_,
                'label': hint.get('label', ''),
                'default': hint.get('default', '{}'),
            })
        return rows

    def actions(self, app, bus, name: str) -> list:
        """노드 메뉴에 붙는 서비스 테스트 항목."""
        if name.rstrip('/').split('/')[-1] != self.node:
            return []

        def enabled_now() -> bool | None:
            """현재 FollowPath.enable — 주행 중이면 status, 아니면 파라미터에서 읽는다."""
            st = bus.state.rl_status
            age = time.monotonic() - bus.state.rl_status_time if st else None
            if st and age is not None and age <= 2.0 and len(st) > 3:
                return st[3] >= 0.5
            target_node = bus.param_target('controller_server')
            if target_node is not None:
                cli = bus.param_client(*target_node)
                if cli is not None:
                    vals = bus.param_get_vals(cli, ['FollowPath.enable'], timeout=1.0)
                    if vals:
                        try:
                            return bool(vals[0].bool_value)
                        except Exception:
                            pass
            for diag_name, values in bus.state.diag_values.items():
                if 'controller_server' in diag_name:
                    return dict(values).get('enable') == 'true'
            return None

        def toggle():
            enabled = enabled_now()
            target = (not enabled) if enabled is not None else True
            now = '?' if enabled is None else ('ON' if enabled else 'OFF')
            body = [f'현재: {now}', f'바꿀 상태: {"ON (정책)" if target else "OFF (fallback)"}',
                    '서비스: /controller_server/FollowPath/toggle']
            if not app.ui.confirm('FollowPath.toggle을 보낼까요?', default=False, body=body):
                return
            ok, result = bus.call_service(
                '/controller_server/FollowPath/toggle', 'std_srvs/srv/SetBool', {'data': target})
            if ok:
                print(f'FollowPath.toggle(data={str(target).lower()})')
                print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
            else:
                print(f'실패: {result}')

        def fallback_now() -> str:
            target_node = bus.param_target('controller_server')
            if target_node is not None:
                cli = bus.param_client(*target_node)
                if cli is not None:
                    vals = bus.param_get_vals(cli, ['FollowPath.fallback_controller'], timeout=1.0)
                    if vals:
                        try:
                            return str(vals[0].string_value)
                        except Exception:
                            pass
            return '?'

        def change_fallback():
            names = {'1': 'mppi', '2': 'dwb'}
            cur = fallback_now()
            choice = app.ui.select(
                'RL 바이패스 controller',
                [('1', 'mppi (기본 · 부드러운 주행)'), ('2', 'dwb (가볍고 단순)')],
                [f'현재: {cur}', 'Nav를 재시작해야 적용됩니다 (주행 중이면 멈춤)',
                 '설정 파일 + inner 프리셋 저장 → tb3_bringup 빌드 → nav restart'])
            if choice not in names:
                return
            name_new = names[choice]
            if name_new == cur:
                print(f'이미 fallback={cur} — 바꾸지 않음')
                return
            if not app.ui.confirm(f'fallback을 {cur} → {name_new}로 바꿀까요?', default=False,
                                  body=['설정: ros2_ws/src/tb3_bringup/config/nav2_params.yaml',
                                        '저장 후 tb3_bringup을 빌드하고 Nav를 재시작합니다.',
                                        '주행 중이면 로봇이 멈춥니다.']):
                return
            app.ui.busy(f'fallback {cur} → {name_new} · 빌드/재시작')
            app.ui.progress(f'{name_new} 설정 저장 중…')
            try:
                ok, msg = bus.set_fallback(name_new)
            finally:
                app.ui._busy = None
            print(('적용: ' if ok else '실패: ') + msg)

        return [
            GoMenu(app, 'a', '서비스 목록 · 호출 (테스트)',
                   ServiceMenu(app, name, self.services(bus, name))),
            ActionItem(app, 'b', '서비스: FollowPath.toggle (RL↔fallback)', toggle),
            ActionItem(app, 'c', '바이패스 controller 변경 (MPPI/DWB) — Nav 재시작', change_fallback),
        ]

    @staticmethod
    def _cmd_hz(bus) -> float:
        return bus.state.cmd_vel_nav.hz()

    def status_lines(self, bus, name: str, summary: bool = False) -> list[str]:
        st = bus.state.rl_status
        age = time.monotonic() - bus.state.rl_status_time if st else None
        cmd_hz = self._cmd_hz(bus)
        fresh = bool(st) and age is not None and age <= 1.0
        try:
            has_status = bus.node.count_publishers('/controller_server/FollowPath/status') > 0
        except Exception:  # fake_ros 처럼 count_publishers가 없는 환경
            has_status = bool(st)

        if summary:
            if fresh:
                inner = INNER.get(int(st[9]) if len(st) > 9 else 0, '기본 제어기')
                mode = 'RL 적용' if st[0] >= 0.5 else f'{inner} 바이패스'
                return [f' RL: {mode} (controller_server)']
            return [f' RL: 대기 (controller_server{"" if has_status else " · status 없음"})']

        lines = ['[RL 주행]']
        if not fresh:
            if cmd_hz > 0:
                lines.append(f' RL/MPPI 판별 불가 (status 없음 · cmd_vel_nav {cmd_hz:.1f}Hz)')
            else:
                lines.append(' 대기 — 주행 중 아님'
                             + (' (status 발행 대기)' if has_status else ' (status 토픽 없음)'))
            if not has_status:
                lines.append(' status 토픽 없음: rl_nav_controller 재빌드 후 controller_server 재시작 필요')
            return lines

        v = list(st) + [0.0] * (10 - len(st))
        rl_active, reason, loaded, enable, scan_ok, scan_age, scan_min, v_out, w_out, inner_kind = v[:10]
        inner = INNER.get(int(inner_kind), '기본 제어기')
        mode = 'RL 적용' if rl_active >= 0.5 else f'{inner} 바이패스'
        why = REASONS.get(int(reason), f'reason {int(reason)}')
        lines.append(f' {mode}' + (f' — {why}' if why else ''))
        lines.append(
            f' 모델 {"loaded" if loaded else "none"} · enable {"on" if enable else "off"}'
            f' · scan {"ok" if scan_ok else "none"}'
            + (f' ({scan_age:.2f}s)' if scan_age >= 0 else ''))
        lines.append(f' cmd_vel_nav {cmd_hz:.1f}Hz · v {v_out:.2f} w {w_out:.2f}')
        if 0.0 < scan_min < 3.49:
            lines.append(f' 전방 최소거리 {scan_min:.2f} m')
        return lines
