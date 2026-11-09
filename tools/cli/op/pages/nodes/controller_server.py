"""controller_server — Nav2 FollowPath(rl_nav_controller::RlNavController)의 RL/MPPI 상태.

플러그인이 매 제어 주기 `/controller_server/FollowPath/status`를 발행한다.
  [rl_active, reason, policy_loaded, enable, scan_ok, scan_age, scan_min, v, w, inner_kind]
  reason: 0=RL, 1=모델 없음/로드 실패, 2=enable=false, 3=/scan 없음, 4=추론 실패
  inner_kind: 1=DWB, 2=MPPI, 3=RPP, 4=RotationShim
"""
from __future__ import annotations

import time

from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray

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
