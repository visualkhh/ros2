"""MapPage — 터미널 지도 화면 (Menu 하위). 중앙 = MapView(포커스 가능), 왼쪽 = 동작 메뉴.

  window.navigate('map', mode=…, source=…, …)  또는  MapPage(app, …)
    mode    'view'  지도 보기만 (기본)
            'pose'  로봇 위치 지정 — 십자선 위치(+방향)를 on_pose(x, y, yaw_deg)로 보냄
            'nav'   한 화면에서 목표·초기위치·취소 — Enter로 웨이포인트, g 주행, x 취소, i 초기위치
            'teleop' 지도 보면서 수동 이동 — WASD/방향키, s 정지, +/- 속도 (MANUAL 필요)
    source  None = 지금 /map(live · 로봇 점 표시) · 경로 = 저장된 지도 yaml(map_server 형식)

열면 지도를 받아 중앙에 올린다 (Window.run의 center_comp) — 처음엔 지도가 포커스를 받아 방향키로 지도 이동 · +/- 확대·축소 ·
Enter = mode의 동작(웨이포인트 추가 · 위치 지정). Tab/Esc로 왼쪽 메뉴(같은 동작)와 오간다. Esc/b = 나가기.
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

from ..defs import BACK, MAIN, QUIT, STAY
from ..dialog import fmt_pose
from ..terminal import DIM, YELLOW
from ..menus import ActionItem, Menu
from ..views.map import FREE, OCC, UNKNOWN, CostGrid, MapGrid, MapView
from ..views.map.map_view import LOCAL_PLAN_RGB, PLAN_RGB

YAW_STEP = 15.0
CELL_TXT = {UNKNOWN: '미확인', FREE: '빈 곳', OCC: '벽'}
TITLES = {
    'view': '지도 보기',
    'pose': '로봇 위치 지정',
    'nav': '지도 주행 — 목표·초기위치·취소',
    'teleop': '지도 보면서 수동 이동 (SLAM)',
}


class MapPage(Menu):
    def __init__(self, app, mode: str = 'view', source=None,
                 goals: list[dict] | None = None, on_pose=None, title: str | None = None):
        super().__init__(app)
        if mode not in TITLES:
            raise ValueError(f'mode는 {"/".join(TITLES)} 중 하나: {mode!r}')
        self.mode = mode
        self.source = Path(source) if source and not isinstance(source, str) else source
        self.on_pose = on_pose
        self.title = title or (TITLES[mode] + (f' · {Path(str(self.source)).parent.name}' if self.source else ''))
        self.goals = [dict(g) for g in (goals or [])]
        self.view: MapView | None = None
        self.yaw = 0.0
        self.msg = ''
        self._robot = None
        self._robot_yaw = None
        self._robot_at = 0.0
        self._odom = None
        self._odom_yaw = None
        self._fitted = False
        self.show_map = False        # m — map 원점 M
        self.show_odom = False       # o — odom 원점 O
        self.snap = True             # s — 벽/costmap 피해서 가까운 빈 칸으로
        self.plan_on = False         # p — Nav2 /plan · /local_plan 겹침
        self.cost_mode = 0           # c — 0 OFF · 1 global · 2 local
        self._preview = None         # v — planner 예상 경로
        self._plans: dict[str, tuple[float, list[tuple[float, float]]] | None] = {
            'plan': None, 'local_plan': None}
        self._map_msg = None
        self._map_msg_at = 0.0
        self._grid_at = 0.0
        self._map_updates = 0
        self.show_scan = False       # l — /scan 라이다 점
        self._scan_points: list[tuple[float, float]] = []
        self._scan_at = 0.0
        self._teleop_move = (0, 0)
        self._teleop_lin, self._teleop_ang = 0.10, 0.50
        self._teleop_at = 0.0
        self._cost: dict[str, CostGrid | None] = {'global': None, 'local': None}
        self._cost_at: dict[str, float] = {'global': 0.0, 'local': 0.0}

    # ---------------------------------------------------------------- 동작 (키 · 메뉴 공용)
    def pick(self) -> None:
        """십자선 위치에서 mode의 동작 — nav: 웨이포인트 추가 · pose: 로봇 위치 지정 · view: 없음."""
        x, y = self.view.center
        if self.mode == 'view':
            self.msg = '보기 전용 — 목표·초기위치는 Nav(4) 또는 지도/위치(6)에서'
        elif self.mode == 'teleop':
            self.msg = 'WASD/방향키 이동 · s 정지 · [ ] 속도 · +/- 지도 · Tab 메뉴'
            return
        elif self.mode == 'nav':
            note = ''
            if self.snap:
                sx, sy, moved = self._safe_point(x, y)
                if moved:
                    x, y, note = sx, sy, ' (costmap/벽 피해서 이동)'
            if self.view.grid.code(self.view.vp.k, x, y) == OCC:
                self.msg = '벽 위에는 못 찍음 — 십자선을 옮기세요'
                return
            self.goals.append({'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)})
            self.msg = (f'웨이포인트 {len(self.goals)}개 추가 {fmt_pose(self.goals[-1])}{note}'
                        ' — g 주행 · i 초기위치 · x 취소')
        elif self.view.grid.code(self.view.vp.k, x, y) == OCC:
            self.msg = '벽 위에는 못 찍음 — 십자선을 옮기세요'
        else:
            self._send_pose(x, y)

    def _send_pose(self, x: float, y: float) -> None:
        """위치 지정(initialpose) — on_pose 콜백이 있으면 그것, 없으면 /initialpose."""
        pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)}
        if not self.ui.confirm('여기를 로봇 위치로 지정할까요?', default=True, body=[fmt_pose(pose)]):
            self.msg = '취소'
            return
        try:
            self.ui.busy('초기 위치 지정')
            self.ui.progress('amcl 반영 기다리는 중…')
            try:
                with self.ui.capture() as (out, err):
                    if self.on_pose is not None:
                        self.on_pose(pose['x'], pose['y'], pose['yaw_deg'])
                    else:
                        self.bus.set_initial_pose(pose['x'], pose['y'], pose['yaw_deg'])
            finally:
                self.ui._busy = None
            self.msg = f'위치 지정 {fmt_pose(pose)}'
        except Exception as exc:
            self.ui._busy = None
            self.msg = f'FAIL: {exc}'

    def set_initial_pose_here(self) -> None:
        """십자선 위치·방향을 로봇 초기 위치로 — /initialpose. 위치를 잃었을 때 여기서 바로."""
        x, y = self.view.center
        pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)}
        if not self.ui.confirm('여기를 로봇 초기 위치로 지정할까요?',
                               body=[fmt_pose(pose), '위치추정(LOCALIZATION)에서 위치를 잃었을 때 쓰세요']):
            self.msg = '취소'
            return
        try:
            self.ui.busy('초기 위치 지정')
            self.ui.progress('로봇 위치 이동 + amcl 반영 중…')
            try:
                with self.ui.capture() as (out, err):
                    rc = self.bus.set_initial_pose(pose['x'], pose['y'], pose['yaw_deg'])
            finally:
                self.ui._busy = None
            text = (out.getvalue() + err.getvalue()).strip().splitlines()
            self.msg = text[-1] if text else f'초기 위치 {fmt_pose(pose)}'
            if rc != 0 and not text:
                self.msg = '초기 위치 실패 — amcl/Nav가 떠 있는지 확인'
        except Exception as exc:
            self.ui._busy = None
            self.msg = f'FAIL: {exc}'

    def undo_goal(self) -> None:
        if self.goals:
            self.goals.pop()
            self.msg = '마지막 웨이포인트 취소'
        else:
            self.msg = '취소할 웨이포인트 없음'

    def clear_goals(self) -> None:
        if self.goals:
            self.goals.clear()
            self.msg = '웨이포인트 전체 지움'
        else:
            self.msg = '지울 웨이포인트 없음'

    def send_waypoints(self) -> None:
        """찍어 둔 웨이포인트를 Nav2에 보낸다 — 1개는 NavigateToPose, 여러 개는 한 번에."""
        if not self.goals:
            self.msg = '웨이포인트 없음 — Enter로 먼저 찍으세요'
            return
        body = [f'{i}) {fmt_pose(g)}' for i, g in enumerate(self.goals, 1)]
        bad = [f'{i}) {fmt_pose(g)}' for i, g in enumerate(self.goals, 1)
               if self._cost_code(g['x'], g['y']) >= 1 or
               self.view.grid.code(self.view.vp.k, g['x'], g['y']) == OCC]
        if bad:
            body.append(self.term.paint('WARN: costmap/벽 위 웨이포인트 — planner가 거부할 수 있음', YELLOW))
            body += bad
        how = '한 점씩' if len(self.goals) == 1 else 'NavigateThroughPoses 한 번에'
        if not self.ui.confirm(f'웨이포인트 {len(self.goals)}개를 {how} 보낼까요?',
                               default=not bad, body=body):
            self.msg = '전송 취소'
            return
        try:
            self.ui.busy('목표 전송')
            self.ui.progress('Nav2 목표 보내는 중…')
            try:
                with self.ui.capture() as (out, err):
                    rc = self.bus.send_waypoints(self.goals)
            finally:
                self.ui._busy = None
            text = (out.getvalue() + err.getvalue()).strip().splitlines()
            last = text[-1] if text else ''
            self.msg = (f'목표 {len(self.goals)}개 전송 — {last}' if rc == 0
                        else f'FAIL: {last or "목표 전송 실패"}')
        except Exception as exc:
            self.ui._busy = None
            self.msg = f'FAIL: {exc}'

    def cancel_nav(self) -> None:
        """주행 취소 + 지도에 남은 웨이포인트 표시도 지운다 (x)."""
        try:
            self.ui.busy('주행 취소')
            try:
                with self.ui.capture() as (out, err):
                    self.bus.cancel_goal()
            finally:
                self.ui._busy = None
            text = (out.getvalue() + err.getvalue()).strip().splitlines()
            self.goals.clear()
            self._preview = None                 # v 예상 경로도 같이 지움
            self._plans = {'plan': None, 'local_plan': None}
            self.plan_on = False                 # p 경로 겹침도 끔
            self.msg = text[-1] if text else 'Nav2 주행 취소'
        except Exception as exc:
            self.ui._busy = None
            self.msg = f'FAIL: {exc}'

    def turn(self, delta: float) -> None:
        self.yaw = (self.yaw + delta + 180) % 360 - 180

    def toggle_follow(self) -> None:
        self.view.following = not self.view.following

    def zoom(self, d: int) -> None:
        if not self.view.vp.zoom(d):
            self.msg = '더 이상 ' + ('확대' if d > 0 else '축소') + '할 수 없음'

    def fit(self) -> None:
        self.view.on_key('f')

    def _odom_origin(self):
        """map 기준 odom 원점·방향 — 같은 순간의 map 자세(amcl)와 odom 자세로 계산."""
        if self.state.amcl is None or self.state.pose is None:
            return None
        xm, ym, tm = self.state.amcl
        xo, yo, to = self.state.pose
        d = tm - to
        c, s = math.cos(d), math.sin(d)
        return (xm - (c * xo - s * yo), ym - (s * xo + c * yo)), math.degrees(d)

    def _on_screen(self, p: tuple[float, float]) -> bool:
        """그 세계 좌표가 지금 지도 화면 안에 있나 — M/O가 안 보이는 이유를 알려 주려고."""
        if self.view is None:
            return False
        cols, rows = self.view.size
        c, r = self.view.vp.to_screen(p[0], p[1], cols, rows * 2)
        return 0 <= c < cols and 0 <= r < rows * 2

    def toggle_map_origin(self) -> None:
        self.show_map = not self.show_map
        self.msg = 'M(map 원점) ' + ('● 표시 (0.00, 0.00)' if self.show_map else '○ 숨김')
        if self.show_map and not self._on_screen((0.0, 0.0)):
            self.msg += ' — 화면 밖, f 전체 보기'

    def toggle_odom(self) -> None:
        self.show_odom = not self.show_odom
        odom = self._odom_origin()
        self.msg = 'O(odom 원점) ' + ('● 표시' if self.show_odom else '○ 숨김')
        if self.show_odom:
            if odom is None:
                self.msg += ' — amcl/odom 대기'
            else:
                self._odom, self._odom_yaw = odom
                self.msg += f' ({odom[0][0]:+.2f}, {odom[0][1]:+.2f})'
                if not self._on_screen(odom[0]):
                    self.msg += ' — 화면 밖, f 전체 보기'

    def toggle_plan(self) -> None:
        if self.source:
            self.msg = '저장 지도에는 Nav2 경로가 없음 — 지금 지도(live)에서만'
            return
        self.plan_on = not self.plan_on
        self.msg = 'Nav2 경로 겹침 ' + ('● ON — /plan(파랑) · /local_plan(보라)' if self.plan_on else '○ OFF')

    COST_LABELS = {0: 'OFF', 1: 'GLOBAL (전역 계획)', 2: 'LOCAL (로봇 주변)'}

    def toggle_cost(self) -> None:
        if self.source:
            self.msg = '저장 지도에는 costmap이 없음 — 지금 지도(live)에서만'
            return
        self.cost_mode = (self.cost_mode + 1) % 3
        self.msg = 'costmap 겹침 ' + self.COST_LABELS[self.cost_mode]
        if self.cost_mode and self._cost_grid() is None:
            self.msg += ' — 아직 수신 없음'

    def _on_cost(self, kind: str, msg) -> None:
        try:
            self._cost[kind] = CostGrid.from_msg(msg)
            self._cost_at[kind] = time.monotonic()
        except Exception:
            pass

    def _cost_grid(self) -> CostGrid | None:
        if self.source or not self.cost_mode:
            return None
        kind = 'global' if self.cost_mode == 1 else 'local'
        grid = self._cost.get(kind)
        if grid is None:
            return None
        if grid.frame_id not in ('', 'map'):
            odom = self._odom_origin()
            if odom is None:
                return None
            grid.set_parent(odom[0][0], odom[0][1], math.radians(odom[1]))
        return grid

    def _cost_code(self, x: float, y: float) -> int:
        """그 점의 costmap 위험도 (0 자유, 1 팽창, 2 치명/내접) — global/local 중 큰 값."""
        worst = 0
        for kind in ('global', 'local'):
            grid = self._cost.get(kind)
            if grid is None:
                continue
            if grid.frame_id not in ('', 'map'):
                odom = self._odom_origin()
                if odom is None:
                    continue
                grid.set_parent(odom[0][0], odom[0][1], math.radians(odom[1]))
            worst = max(worst, grid.code(x, y))
        return worst

    def _safe_point(self, x: float, y: float) -> tuple[float, float, bool]:
        """s — 찍은 점이 벽/costmap 팽창·치명이면 가장 가까운 안전한 빈 칸으로."""
        def safe(px, py):
            return (self.view.grid.code(self.view.vp.k, px, py) == FREE
                    and self._cost_code(px, py) == 0)
        if safe(x, y):
            return x, y, False
        step = max(0.05, self.view.grid.res)
        for r in range(1, 31):
            radius = r * step
            for i in range(16):
                a = 2.0 * math.pi * i / 16.0
                px, py = x + radius * math.cos(a), y + radius * math.sin(a)
                if safe(px, py):
                    return px, py, True
        return x, y, False

    def toggle_snap(self) -> None:
        self.snap = not self.snap
        self.msg = ('웨이포인트 안전 스냅 ● 켜짐 — 벽/costmap 팽창을 피해 가장 가까운 빈 칸으로'
                    if self.snap else '웨이포인트 안전 스냅 ○ 꺼짐 — 찍은 자리 그대로')

    def preview_path(self) -> None:
        """v — 찍어 둔 웨이포인트로 planner에 경로만 계산 (로봇은 움직이지 않음)."""
        if not self.goals:
            self.msg = '웨이포인트 없음 — Enter로 먼저 찍으세요'
            return
        if self.source:
            self.msg = '저장 지도에는 planner가 없음 — 지금 지도(live)에서만'
            return
        try:
            self.ui.busy('예상 경로 계산')
            self.ui.progress('Nav2 planner에 경로 요청 중… (로봇은 움직이지 않음)')
            try:
                pts = self.bus.preview_path(
                    self.goals, on_wait=lambda sec: self.ui.progress(f'Nav2 planner 계산 중… {sec:.1f}s'))
            finally:
                self.ui._busy = None
            if len(pts) < 2:
                self.msg = 'planner가 경로를 내지 않음 (출발/목표 확인)'
                return
            self._preview = pts
            self.msg = f'예상 경로 {len(pts)}점 (초록) — 전송은 g'
        except Exception as exc:  # noqa: BLE001 — 화면에 이유를 남긴다
            self.ui._busy = None
            self.msg = f'FAIL: {exc}'

    def clear_preview(self) -> None:
        if self._preview is None:
            self.msg = '예상 경로 없음'
            return
        self._preview = None
        self.msg = '예상 경로 지움'

    def _on_path(self, kind: str, msg) -> None:
        if getattr(getattr(msg, 'header', None), 'frame_id', '') not in ('map', ''):
            return
        self._plans[kind] = (time.monotonic(),
                             [(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def _plan_lines(self):
        """(점들, 색) 목록 — plan_on일 때 15초 안에 받은 /plan·/local_plan만."""
        if not self.plan_on or self.source:
            return []
        now, out = time.monotonic(), []
        for kind, rgb in (('plan', PLAN_RGB), ('local_plan', LOCAL_PLAN_RGB)):
            got = self._plans.get(kind)
            if got and now - got[0] <= 15.0 and len(got[1]) > 1:
                out.append((got[1], rgb))
        return out

    def _on_map(self, msg) -> None:
        self._map_msg = msg
        self._map_msg_at = time.monotonic()

    def _refresh_grid(self) -> None:
        """SLAM live /map — 0.5초에 한 번 지도 격자를 새로 만든다 (뷰·로봇 위치는 유지)."""
        if self.source or self.view is None or self._map_msg is None:
            return
        now = time.monotonic()
        if now - self._grid_at < 0.5:
            return
        try:
            grid = MapGrid.from_msg(self._map_msg)
        except Exception:
            return
        self._map_msg = None
        self._grid_at = now
        self.view.grid = grid
        self.view.vp.grid = grid
        self.view._key = None
        self._map_updates += 1

    def _teleop_cmd(self) -> tuple[float, float]:
        f, t = self._teleop_move
        return f * self._teleop_lin, t * self._teleop_ang

    def _tick(self, seconds: float) -> None:
        """지도 화면이 떠 있는 동안 텔레오퍼 명령을 10Hz로 보낸다."""
        if self.mode != 'teleop':
            return
        now = time.monotonic()
        if now - self._teleop_at < 0.1:
            return
        self._teleop_at = now
        if self.ui.window.focus is not self.view:
            self._teleop_move = (0, 0)
        self.bus.send_velocity(*self._teleop_cmd())

    def _move(self, f: int, t: int) -> None:
        self._teleop_move = (f, t)

    def _stop(self) -> None:
        self._teleop_move = (0, 0)
        self.bus.send_velocity(0.0, 0.0)

    def _sensor_pose(self, frame: str):
        """map 기준 라이다 자세 (x, y, yaw[rad]) — TF, 없으면 로봇 자세로 근사."""
        frame = (frame or '').strip()
        buf = getattr(self.bus.node, 'tf_buffer', None) if hasattr(self.bus, 'node') else None
        if buf is not None and frame:
            try:
                import rclpy
                tr = buf.lookup_transform('map', frame, rclpy.time.Time())
                p, q = tr.transform.translation, tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return p.x, p.y, yaw
            except Exception:
                pass
        if self._robot is None:
            self._pose()
        if self._robot is None:
            return None
        return self._robot[0], self._robot[1], math.radians(self._robot_yaw or 0.0)

    def _on_scan(self, msg) -> None:
        """LaserScan → map 좌표 점 (l로 켰을 때만)."""
        if not self.show_scan or self.source:
            self._scan_points = []
            return
        pose = self._sensor_pose(getattr(getattr(msg, 'header', None), 'frame_id', ''))
        if pose is None:
            return
        x0, y0, yaw = pose
        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment + yaw
            pts.append((x0 + r * math.cos(a), y0 + r * math.sin(a)))
        self._scan_points = pts
        self._scan_at = time.monotonic()

    def toggle_scan(self) -> None:
        if self.source:
            self.msg = '저장 지도에는 라이다 없음 — 지금 지도(live)에서만'
            return
        self.show_scan = not self.show_scan
        if not self.show_scan:
            self._scan_points = []
            self.msg = '라이다 스캔 ○ 꺼짐'
        else:
            self.msg = '라이다 스캔 ● 켜짐 — /scan 점을 빨간색으로 (RViz LaserScan)'

    def _extra_key(self, k) -> bool:
        """지도가 모르는 키 — teleop은 이동, 그 외는 목표·초기위치·화면 옵션."""
        if self.mode == 'teleop':
            moves = {'w': (1, 0), 'up': (1, 0), 'x': (-1, 0), 'down': (-1, 0),
                     'a': (0, 1), 'left': (0, 1), 'd': (0, -1), 'right': (0, -1),
                     's': (0, 0), ' ': (0, 0)}
            if k in moves:
                self._teleop_move = moves[k]
                self.msg = (f'텔레오퍼 v={self._teleop_cmd()[0]:+.2f} m/s '
                            f'ω={self._teleop_cmd()[1]:+.2f} rad/s')
                return True
            if k in (']', '}', '.', '>'):   # + / - 는 지도 확대·축소 — 속도는 [ ]
                self._teleop_lin = min(0.26, self._teleop_lin + 0.02)
                self._teleop_ang = min(1.82, self._teleop_ang + 0.1)
                self.msg = f'속도 v {self._teleop_lin:.2f} · ω {self._teleop_ang:.2f}'
                return True
            if k in ('[', '{', ',', '<'):
                self._teleop_lin = max(0.05, self._teleop_lin - 0.02)
                self._teleop_ang = max(0.2, self._teleop_ang - 0.1)
                self.msg = f'속도 v {self._teleop_lin:.2f} · ω {self._teleop_ang:.2f}'
                return True
            # m·o·p·c는 아래 공통 지도 옵션으로 넘긴다 (텔레오퍼 키와 겹치지 않음)
        if k in ('enter', ' '):
            self.pick()
        elif k == 'backspace' and self.mode == 'nav':
            self.undo_goal()
        elif k in ('g', 'G') and self.mode == 'nav':
            self.send_waypoints()
        elif k in ('d', 'D') and self.mode == 'nav':
            self.clear_goals()
        elif k in ('x', 'X') and self.mode == 'nav':
            self.cancel_nav()
        elif k in ('i', 'I') and self.mode == 'nav':
            self.set_initial_pose_here()
        elif k in ('v', 'V') and self.mode == 'nav':
            if k == 'v':
                self.preview_path()
            else:
                self.clear_preview()
        elif k in ('p', 'P'):
            self.toggle_plan()
        elif k in ('c', 'C'):
            self.toggle_cost()
        elif k in ('l', 'L'):
            self.toggle_scan()
        elif k in ('s', 'S') and self.mode == 'nav':
            self.toggle_snap()
        elif k in ('m', 'M'):
            self.toggle_map_origin()
        elif k in ('o', 'O'):
            self.toggle_odom()
        elif k in ('[', ']', '{', '}') and self.mode != 'view':   # [ ] = 15° · { } = 1° (미세)
            self.turn({'[': YAW_STEP, ']': -YAW_STEP, '{': 1.0, '}': -1.0}[k])
        else:
            return False
        return True

    # ---------------------------------------------------------------- 화면
    def items(self):
        v, act = self.view, ActionItem
        rows = {
            'nav': [('웨이포인트 추가 — 십자선 위치 (Enter)', self.pick),
                    ('마지막 웨이포인트 취소 (Backspace)', self.undo_goal),
                    ('목표로 주행 — 한 점/여러 점 (g)', self.send_waypoints),
                    ('웨이포인트 전체 지우기 (d)', self.clear_goals),
                    ('로봇 초기 위치 지정 — 십자선 (i)', self.set_initial_pose_here),
                    ('Nav2 주행 취소 (x)', self.cancel_nav),
                    ('예상 경로 미리보기 — 계산만 (v)', self.preview_path),
                    ('예상 경로 지우기 (V)', self.clear_preview),
                    (lambda: f'웨이포인트 안전 스냅 (s) — {"●" if self.snap else "○"}', self.toggle_snap)],
            'pose': [('로봇 위치로 지정 — 십자선 위치 (Enter)', self.pick)],
            'teleop': [('지도에 포커스한 채 WASD/방향키로 이동 · s 정지', lambda: None),
                       ('속도는 지도에서 [ / ] (기본 0.10 m/s · 0.50 rad/s)', lambda: None)],
            'view': [],
        }[self.mode]
        rows += [('확대 (+)', lambda: self.zoom(+1)),
                 ('축소 (-)', lambda: self.zoom(-1)),
                 ('전체 보기 (f)', self.fit),
                 (lambda: f'로봇 따라가기 (r) — {"켜짐" if v.following else "꺼짐"}', self.toggle_follow),
                 (lambda: f'M map 원점 (m) — {"●" if self.show_map else "○"}', self.toggle_map_origin),
                 (lambda: f'O odom 원점 (o) — {"●" if self.show_odom else "○"}', self.toggle_odom),
                 (lambda: f'Nav2 경로 겹쳐 보기 (p) — {"●" if self.plan_on else "○"}', self.toggle_plan),
                 (lambda: f'costmap 겹쳐 보기 (c) — {self.COST_LABELS[self.cost_mode]}', self.toggle_cost),
                 (lambda: f'라이다 스캔 겹쳐 보기 (l) — {"●" if self.show_scan else "○"}', self.toggle_scan)]
        if self.mode in ('nav', 'pose'):
            rows += [(f'좌회전 +{YAW_STEP:.0f}° ([) · 미세 +1° = {{', lambda: self.turn(YAW_STEP)),
                     (f'우회전 -{YAW_STEP:.0f}° (]) · 미세 -1° = }}', lambda: self.turn(-YAW_STEP))]
        return [act(self.app, str(i), label, fn) for i, (label, fn) in enumerate(rows, 1)]

    def _pose(self):
        """로봇 자세 — 0.3초에 한 번만 (막히지 않는 조회). 저장된 지도 파일을 볼 땐 로봇 점 없음."""
        if self.source:
            return None
        now = time.monotonic()
        interval = 0.1 if self.mode == 'teleop' else 0.3   # 텔레오퍼는 방향 화살표가 빨리 따라오게
        if now - self._robot_at >= interval:
            self._robot_at = now
            p = self.bus.robot_pose()
            self._robot = (p['x'], p['y']) if p else None
            self._robot_yaw = p['yaw_deg'] if p else None
        return self._robot

    def center(self) -> list[str]:
        self._refresh_grid()
        v = self.view
        cw, ch = self.ui.window.center_size()
        rows = max(3, ch - 9)   # 아래 상태·메시지·범례까지 화면에 남긴다
        v.show_yaw = self.mode in ('nav', 'pose')
        v.show_cursor = self.mode in ('nav', 'pose')
        v.teleop = self.mode == 'teleop'
        v.robot = self._pose()
        v.robot_yaw = self._robot_yaw
        odom = self._odom_origin()
        if odom is not None:
            self._odom, self._odom_yaw = odom
        v.odom, v.odom_yaw = self._odom, self._odom_yaw
        v.show_map, v.show_odom = self.show_map, self.show_odom
        v.preview = self._preview or []
        v.plans = self._plan_lines()
        v.cost = self._cost_grid()
        v.show_cost = v.cost is not None
        v.scan, v.show_scan = self._scan_points, self.show_scan
        v.start = None
        v.goals, v.yaw = self.goals, self.yaw
        if not self._fitted:
            v.vp.fit(cw, rows * 2)
            self._fitted = True
        lines = v.render(cw, rows)
        x, y = v.center
        code = v.grid.code(v.vp.k, x, y)
        if self.mode == 'nav':
            mode_txt = f'방향 {self.yaw:+.0f}° · 웨이포인트 {len(self.goals)}개'
        elif self.mode == 'teleop':
            tv, tw = self._teleop_cmd()
            mode_txt = f'텔레오퍼 v={tv:+.2f} m/s ω={tw:+.2f} rad/s'
        elif self.mode == 'pose':
            mode_txt = f'방향 {self.yaw:+.0f}°'
        else:
            mode_txt = '보기 전용'
        point_name = '십자선' if self.mode in ('nav', 'pose') else '화면 중심'
        status = [f'{point_name} ({x:+.2f}, {y:+.2f}) m · {CELL_TXT.get(code, "지도 밖")}',
                  f'한 칸 = {v.vp.scale * 100:.0f} cm · {"따라가기" if v.following else "자유이동"}',
                  mode_txt]
        if v.robot and v.robot_yaw is not None:
            status.append(f'로봇 ({v.robot[0]:+.2f}, {v.robot[1]:+.2f}) m · {v.robot_yaw:+.0f}°')
        if self._preview:
            status.append(f'예상 경로 {len(self._preview)}점 (초록)')
        if self.plan_on:
            plans = self._plan_lines()
            status.append('Nav2 경로 ' + (f'{sum(len(p) for p, _ in plans)}점 (파랑/보라)' if plans else '대기'))
        if self.cost_mode:
            status.append('costmap ' + self.COST_LABELS[self.cost_mode]
                          + ('' if self._cost_grid() is not None else ' — 수신 대기'))
        if self.show_scan:
            age = time.monotonic() - self._scan_at if self._scan_at else None
            status.append(f'라이다 {len(self._scan_points)}점' + (
                '' if age is not None and age <= 2.0 else ' — /scan 대기'))
        mo = [f'M map {"●" if self.show_map else "○"}']
        if self.show_map:
            mo.append('(0.00,0.00)')
        mo.append(f'O odom {"●" if self.show_odom else "○"}')
        if self.show_odom and self._odom:
            mo.append(f'({self._odom[0]:+.2f},{self._odom[1]:+.2f})')
        status.append(' '.join(mo))
        if not self.source:
            age = time.monotonic() - self._map_msg_at if self._map_msg_at else None
            live = '● 갱신 중' if age is not None and age <= 2.0 else '○ 대기'
            status.append(f'live /map {live} · {v.grid.w}x{v.grid.h} · {self._map_updates}회')
        if self.state.goal_status:
            status.append(f'목표: {self.state.goal_status}')
        legend_txt = '● 로봇 → 방향'
        if self.mode in ('nav', 'pose'):
            legend_txt += ' · + 십자선'
        legend_txt += ' · S 출발 · 1 2 goal · M map · O odom'
        legend = self.term.paint(legend_txt, DIM)
        return lines + status + [self.msg, legend]

    # ---------------------------------------------------------------- 실행
    def _load_grid(self, timeout: float = 10.0):
        if self.source:
            # PC에서 이미 내려받은 캐시 경로면 그대로, 로봇 경로면 local_map이 내려받는다.
            p = Path(self.source).expanduser()
            local = p if p.is_file() else self.bus.procs.local_map(self.source)
            if local is None:
                raise RuntimeError(f'지도 파일을 못 가져옴: {self.source} — 로봇 경로/SSH 확인')
            return MapGrid.from_yaml(local)
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        inbox: list = []
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.app.create_subscription(OccupancyGrid, '/map', inbox.append, qos)
        try:
            end = time.monotonic() + timeout
            while not inbox and time.monotonic() < end:
                self.ui.progress(f'/map 기다리는 중 … {max(0, end - time.monotonic()):.0f}초')
                self.app.poll(0.2)
            return MapGrid.from_msg(inbox[-1]) if inbox else None
        finally:
            self.app.destroy_subscription(sub)
            self.ui._busy = None

    @staticmethod
    def _warm(grid: MapGrid) -> None:
        """축소 단계(벽 우선 피라미드)를 미리 다 만들어 둔다 — 열린 뒤 확대·축소가 끊기지 않게."""
        k = 0
        while grid.level(k)[:2] != (1, 1):
            k += 1

    def show(self) -> str:
        if self.mode == 'teleop' and not self.bus.ensure_manual():
            return STAY
        self.ui.busy('지도 여는 중 …')
        try:
            grid = self._load_grid()
            if grid is not None:
                self._warm(grid)
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return STAY
        finally:
            self.ui._busy = None
        if grid is None:
            print('FAIL: /map을 못 받음 — 지도 서버(map_server)·Nav가 떠 있는지 확인', file=sys.stderr)
            return STAY
        self.view = MapView(grid, self.term.color, extra_key=self._extra_key)
        self.view.teleop = self.mode == 'teleop'
        self._plans = {'plan': None, 'local_plan': None}
        self._cost = {'global': None, 'local': None}
        self._cost_at = {'global': 0.0, 'local': 0.0}
        self._map_msg = None
        self._grid_at = 0.0
        self._map_updates = 0
        self._scan_points = []
        self._scan_at = 0.0
        subs = []
        try:
            from nav_msgs.msg import Path, OccupancyGrid
            for topic, kind in (('/plan', 'plan'), ('/local_plan', 'local_plan')):
                subs.append(self.app.create_subscription(
                    Path, topic, lambda m, k=kind: self._on_path(k, m), 10))
            for topic, kind in (('/global_costmap/costmap', 'global'),
                                ('/local_costmap/costmap', 'local')):
                subs.append(self.app.create_subscription(
                    OccupancyGrid, topic, lambda m, k=kind: self._on_cost(k, m), 10))
            if not self.source:
                from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                                       qos_profile_sensor_data)
                qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
                subs.append(self.app.create_subscription(OccupancyGrid, '/map', self._on_map, qos))
                from sensor_msgs.msg import LaserScan
                subs.append(self.app.create_subscription(
                    LaserScan, '/scan', self._on_scan, qos_profile_sensor_data))
        except Exception:
            pass
        tick = self.mode == 'teleop'
        if tick:
            self.view.hints = 'WASD/방향키 이동 · s 정지 · [ ] 속도 · +/- 지도 · Tab 메뉴 · Esc 메뉴'
            self.ui.window.connect('tick', self._tick)
        try:
            result = self.ui.window.run(self.title, self.items, self.center, back=True, main=True,
                                        center_comp=self.view, focus_center=True)
        finally:
            if tick:
                self.ui.window.disconnect('tick', self._tick)
                for _ in range(3):
                    self.bus.send_velocity(0.0, 0.0)
                    self.bus._poll(0.05)
            for sub in subs:
                try:
                    self.app.destroy_subscription(sub)
                except Exception:
                    pass
        return result if result in (MAIN, QUIT) else BACK
