"""StateStore — 구독한 ROS 상태 보관. 화면은 여기만 읽는다 (ROS 타입을 모름)."""
from __future__ import annotations

import collections
import math
import time


class Rate:
    """최근 수신 시각으로 Hz·마지막 수신 경과를 계산."""

    def __init__(self, window: int = 20):
        self.stamps: collections.deque[float] = collections.deque(maxlen=window)

    def tick(self) -> None:
        self.stamps.append(time.monotonic())

    def age(self) -> float | None:
        return time.monotonic() - self.stamps[-1] if self.stamps else None

    def hz(self) -> float:
        if len(self.stamps) < 2 or (self.age() or 0) > 3.0:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0


class StateStore:
    def __init__(self):
        self.odom = Rate()
        self.scan = Rate()
        self.map = Rate(5)
        self.pose: tuple[float, float, float] | None = None       # odom x, y, yaw
        self.vel: tuple[float, float] | None = None               # v, ω
        self.amcl: tuple[float, float, float] | None = None       # map 기준 x, y, yaw
        self.amcl_rate = Rate(5)
        self.scan_info = ''                                        # 점 수 · 범위
        self.map_info: tuple[int, int, float] | None = None       # w, h, res
        self.map_known = 0.0                                       # 알려진 칸 비율
        self.goal_status = ''                                      # 마지막 목표 진행
        self.robot = Rate(5)                                       # /diagnostics 'opencr' 수신
        self.robot_level = -1                                      # 0 OK · 1 WARN · 2 ERROR · -1 모름
        self.robot_msg = ''
        self.robot_vals: dict[str, str] = {}                       # 전류·온도·uptime·jumps
        self.last_cmd: tuple[float, float] = (0.0, 0.0)
        self.st_control: str | None = None                            # state_manager /state_* (None = 수신 없음)
        self.st_rl: bool | None = None
        self.st_estop: bool | None = None
        self.st_operation: str | None = None                       # 운행 상태 SLAM · LOCALIZATION · IDLE
        self.st_map: str | None = None                             # 적용된 지도
        self.explore_status = ''                                   # explore_bridge /explore_status
        self.rl_out = Rate(5)                                      # /cmd_vel_rl_out (PC RL 응답)

    @staticmethod
    def yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_odom(self, msg) -> None:
        self.odom.tick()
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, self.yaw(p.orientation))
        self.vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_scan(self, msg) -> None:
        self.scan.tick()
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        near = f' · 최근접 {min(valid):.2f}m' if valid else ''
        self.scan_info = f'{len(valid)}/{len(msg.ranges)}점{near}'

    def on_map(self, msg) -> None:
        self.map.tick()
        info = msg.info
        self.map_info = (info.width, info.height, info.resolution)
        n = len(msg.data)
        self.map_known = (sum(1 for v in msg.data if v >= 0) / n) if n else 0.0

    def on_amcl(self, msg) -> None:
        self.amcl_rate.tick()
        p = msg.pose.pose
        self.amcl = (p.position.x, p.position.y, self.yaw(p.orientation))

    # ------------------------------------------------------------ 표시용 문자열
    @staticmethod
    def _age_txt(rate: Rate) -> str:
        age = rate.age()
        if age is None:
            return '수신 없음'
        if rate.hz() > 0:
            return f'{rate.hz():.1f}Hz'
        return f'{age:.0f}초 전'

    def odom_txt(self) -> str:
        if self.pose is None:
            return 'odom 수신 없음'
        x, y, th = self.pose
        return f'x={x:+.2f} y={y:+.2f} θ={math.degrees(th):+.0f}° ({self._age_txt(self.odom)})'

    def scan_txt(self) -> str:
        return f'{self.scan_info} ({self._age_txt(self.scan)})' if self.scan_info else 'scan 수신 없음'

    def map_txt(self) -> str:
        if self.map_info is None:
            return '/map 수신 없음'
        w, h, res = self.map_info
        return (f'{w}x{h} @{res:.2f}m = {w * res:.1f}x{h * res:.1f}m · '
                f'탐색 {self.map_known * 100:.0f}% ({self._age_txt(self.map)})')

    def on_diagnostics(self, msg) -> None:
        for st in msg.status:
            if st.name != 'opencr':
                continue
            self.robot.tick()
            lv = st.level
            self.robot_level = lv[0] if isinstance(lv, (bytes, bytearray)) else int(lv)
            self.robot_msg = st.message
            self.robot_vals = {kv.key: kv.value for kv in st.values}

    def robot_parts(self) -> tuple[int, str]:
        """(레벨, 요약) — 레벨 -1 = 수신 없음/끊김. 요약: 메시지 · 온도 · 전류."""
        age = self.robot.age()
        if age is None:
            return -1, '로봇 상태 대기 중 (1초 주기 · 계속 없으면 베이스 꺼짐?)'
        if age > 3.0:
            return -1, f'로봇 {age:.0f}초 끊김'
        v = self.robot_vals

        def f(key, fmt):
            try:
                return fmt.format(float(v[key]))
            except (KeyError, ValueError):
                return '?'
        temp = f"{f('left_temp_C', '{:.0f}')}/{f('right_temp_C', '{:.0f}')}℃"
        cur = f"{f('left_current_A', '{:.2f}')}/{f('right_current_A', '{:.2f}')}A"
        bat = f"{f('battery_V', '{:.1f}')}V"
        return self.robot_level, f'{self.robot_msg} · 온도 좌/우 {temp} · 전류 {cur} · 배터리 {bat}'

    def robot_txt(self) -> str:
        return self.robot_parts()[1]

    def route_txt(self) -> str:
        if not self.st_rl:
            return '경로 직결'
        age = self.rl_out.age()
        return '경로 RL 경유' + ('' if age is not None and age < 1.0 else ' (PC RL 응답 없음 → 정지 · PC에서 rl-only.launch.py 켜기)')

    def domain_txt(self) -> str:
        if self.st_control is None:
            return '상태 없음 (베이스 꺼짐?)'
        op = self.st_operation or '?'
        if self.st_map:
            op += f' (지도 {self.st_map})'
        txt = f'제어 {self.st_control} · 운행 {op} · {self.route_txt()} · 비상정지 {"ON" if self.st_estop else "off"}'
        if self.st_control == 'SLAM_AUTO_MAPPING' and self.explore_status:
            txt += f' · 탐색 {self.explore_status}'
        return txt

    def amcl_txt(self) -> str:
        if self.amcl is None:
            return 'amcl_pose 없음'
        x, y, th = self.amcl
        return f'x={x:+.2f} y={y:+.2f} θ={math.degrees(th):+.0f}° ({self._age_txt(self.amcl_rate)})'
