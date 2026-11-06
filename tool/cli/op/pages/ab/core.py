"""rl_nav_controller A/B — Nav2 FollowPath RL 정책 ON/OFF · rosbag 기록.

UI 없이 동작만 담당한다. 진입점은 `run-cli ab ...` / 메뉴 `b) A/B`.
  - ON/OFF: /controller_server/FollowPath/toggle (std_srvs/SetBool)
  - 기록: ros2 bag record를 백그라운드로 띄우고 ~/.cache/robot_tool/ab.json에 pid·bag 경로를 남긴다
  - 상태: /controller_server/FollowPath/status (ControllerServerFeature가 state.rl_status에 저장)
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy

AB_NODES = (
    ('rl_nav_controller', 'Nav2 FollowPath (RL vs inner controller)'),
)

NODE = 'rl_nav_controller'
TOGGLE = '/controller_server/FollowPath/toggle'
TOOL_DIR = Path(__file__).resolve().parents[4]   # tool/
BAG_DIR = TOOL_DIR / 'bags' / 'ab'
STATE = Path.home() / '.cache' / 'robot_tool' / 'ab.json'
RECORD_LOG = BAG_DIR / 'record.log'
PLAN = Path(__file__).resolve().parent / NODE / 'command.json'
INNER = {1: 'DWB', 2: 'MPPI', 3: 'RPP', 4: 'RotationShim'}
REASONS = {0: '', 1: '모델 없음/로드 실패', 2: 'enable=false', 3: '/scan 없음', 4: '추론 실패'}
LEGACY_ACTIONS = ('status', 'on', 'off', 'record', 'stop', 'guide', 'history')

# controller 출력 + 실제 움직임 + Nav2 계획/TF
TOPICS = ('/cmd_vel_nav', '/cmd_vel', '/odometry/filtered', '/odom', '/imu',
          '/plan', '/tf', '/tf_static', '/controller_server/FollowPath/status')


class AbTest:
    def __init__(self, bus):
        self.bus = bus
        self.node = bus.node
        self.state = bus.state
        self.term = bus.term
        self._toggle_cli = None
        self._cfg: tuple[str, bool] | None = None
        self._cfg_at = 0.0

    # --------------------------------------------------------------- 상태 파일
    @staticmethod
    def _load() -> dict:
        try:
            data = json.loads(STATE.read_text())
            if isinstance(data, dict):
                data.setdefault('recording', None)
                data.setdefault('history', [])
                return data
        except Exception:
            pass
        return {'recording': None, 'history': []}

    @staticmethod
    def _save(data: dict) -> None:
        STATE.parent.mkdir(parents=True, exist_ok=True)
        tmp = STATE.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1))
        tmp.replace(STATE)

    @staticmethod
    def _alive(pid) -> bool:
        try:
            pid = int(pid)
            os.kill(pid, 0)
        except (TypeError, ValueError, ProcessLookupError, PermissionError):
            return False
        try:
            cmd = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            return True
        return 'bag' in cmd and 'record' in cmd

    def _finalize(self, data: dict, rec: dict, stopped: float | None = None) -> dict:
        rec = dict(rec)
        rec.setdefault('stopped', stopped or time.time())
        hist = [h for h in data.get('history', []) if h.get('bag') != rec.get('bag')]
        hist.append(rec)
        data['history'] = hist[-50:]
        if data.get('recording') and data['recording'].get('bag') == rec.get('bag'):
            data['recording'] = None
        self._save(data)
        return rec

    def current(self) -> dict | None:
        data = self._load()
        rec = data.get('recording')
        if not rec:
            return None
        if self._alive(rec.get('pid')):
            return rec
        self._finalize(data, rec, stopped=rec.get('stopped') or time.time())
        return None

    def history(self) -> list[dict]:
        data = self._load()
        hist = [h for h in data.get('history', []) if h.get('bag') and Path(h['bag']).exists()]
        return hist[-50:]

    # --------------------------------------------------------------- 상태
    def status(self) -> dict:
        st = self.state.rl_status
        fresh = bool(st) and time.monotonic() - self.state.rl_status_time <= 1.0
        if not fresh:
            return {'fresh': False, 'enabled': None, 'model': '?', 'inner': '?', 'reason': None}
        v = list(st) + [0.0] * (10 - len(st))
        return {
            'fresh': True,
            'enabled': v[0] >= 0.5,
            'reason': int(v[1]),
            'model': 'loaded' if v[2] else 'none',
            'scan': v[4] >= 0.5,
            'inner': INNER.get(int(v[9]), '?'),
            'cmd_hz': self.state.cmd_vel_nav.hz(),
        }

    def configured(self) -> tuple[str, bool] | None:
        """주행 전에도 controller_server 파라미터로 모델/enable 확인 (5초 캐시)."""
        now = time.monotonic()
        if now - self._cfg_at < 5.0:
            return self._cfg
        self._cfg_at = now
        self._cfg = None
        try:
            target = self.bus.param_target('controller_server')
            if not target:
                return None
            cli = self.bus.param_client(*target)
            if cli is None or not cli.services_are_ready():
                return None
            vals = self.bus.param_get_vals(cli, ['FollowPath.model_path', 'FollowPath.enable'])
            if vals and len(vals) >= 2:
                model = vals[0].string_value or '(자동 share)'
                self._cfg = (model, bool(vals[1].bool_value))
        except Exception:
            pass
        return self._cfg

    def status_lines(self) -> list[str]:
        st = self.status()
        lines = ['[RL/fallback A/B · rl_nav_controller]']
        if not st['fresh']:
            cfg = self.configured()
            if cfg:
                model, enable = cfg
                lines.append(' 대기 — active goal 없음 (goal을 보내면 RL 상태 표시)')
                lines.append(f' 설정: 모델 {Path(model).name if model else "없음"}'
                             f' · enable {"on" if enable else "off"}')
            else:
                lines.append(' 상태 없음 — controller_server가 꺼졌거나 파라미터 응답 없음')
        else:
            mode = 'RL 적용' if st['enabled'] else f'{st["inner"]} 바이패스'
            why = REASONS.get(st['reason'], f'reason {st["reason"]}')
            lines.append(f' 모드: {mode}' + (f' — {why}' if why else ''))
            lines.append(f' 모델: {st["model"]} · inner {st["inner"]}'
                         f' · scan {"ok" if st["scan"] else "none"}'
                         f' · cmd_vel_nav {st["cmd_hz"]:.1f}Hz')
        rec = self.current()
        lines.append(f' 녹화: {rec["bag"] if rec else "없음"}')
        try:
            plan = self.load_plan()
            lines.append(f' 계획: route {len(plan["route"])}개 · repeats {plan["repeats"]} · {PLAN.name}')
        except RuntimeError as exc:
            lines.append(f' 계획: {exc}')
        hist = self.history()
        if hist:
            lines.append(f' 최근 녹화 {len(hist)}개 · 마지막 {Path(hist[-1]["bag"]).name}')
        return lines

    # --------------------------------------------------------------- toggle
    def _toggle_client(self):
        if self._toggle_cli is None:
            from std_srvs.srv import SetBool
            self._toggle_cli = self.node.create_client(SetBool, TOGGLE)
        return self._toggle_cli

    def toggle(self, on: bool) -> tuple[bool, str]:
        from std_srvs.srv import SetBool
        cli = self._toggle_client()
        if not cli.wait_for_service(timeout_sec=2.0):
            return False, f'{TOGGLE} 없음 — controller_server/Nav2 확인'
        fut = cli.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, 'toggle 응답 없음'
        res = fut.result()
        return bool(res.success), str(res.message)

    # --------------------------------------------------------------- rosbag
    def start(self, label: str | None = None) -> dict:
        if self.current():
            raise RuntimeError('이미 녹화 중 — 먼저 stop')
        BAG_DIR.mkdir(parents=True, exist_ok=True)
        name = f'{time.strftime("%Y-%m-%d_%H-%M-%S")}_{label or "run"}'
        bag = BAG_DIR / name
        RECORD_LOG.parent.mkdir(parents=True, exist_ok=True)
        log = open(RECORD_LOG, 'ab')
        cmd = ['ros2', 'bag', 'record', '-o', str(bag), '-a']
        try:
            proc = subprocess.Popen(cmd, stdout=log, stderr=log, start_new_session=True)
        except OSError as exc:
            log.close()
            raise RuntimeError(f'ros2 bag record 시작 실패: {exc}') from exc
        rec = {'label': label or 'run', 'bag': str(bag), 'pid': proc.pid,
               'started': time.time(), 'cmd': cmd}
        data = self._load()
        data['recording'] = rec
        self._save(data)
        time.sleep(2.0)   # rosbag2가 토픽을 발견할 시간 — 짧은 주행에서 odom/cmd 누락 방지
        return rec

    def stop(self) -> dict | None:
        data = self._load()
        rec = data.get('recording')
        if not rec:
            return None
        pid = rec.get('pid')
        try:
            os.killpg(int(pid), signal.SIGINT)
        except (ProcessLookupError, TypeError, ValueError):
            pass
        end = time.monotonic() + 10.0
        while time.monotonic() < end and self._alive(pid):
            time.sleep(0.1)
        if self._alive(pid):
            try:
                os.killpg(int(pid), signal.SIGKILL)
            except (ProcessLookupError, TypeError, ValueError):
                pass
        return self._finalize(data, rec, stopped=time.time())

    # --------------------------------------------------------------- 자동 A/B
    def load_plan(self, path: str | None = None) -> dict:
        p = Path(path).expanduser() if path else PLAN
        try:
            data = json.loads(p.read_text())
        except (OSError, ValueError) as exc:
            raise RuntimeError(f'계획 파일을 못 읽음 ({p}): {exc!r}') from exc
        route = data.get('route') or []
        if not route:
            raise RuntimeError(f'계획에 route가 없음: {p}')
        for i, g in enumerate(route):
            if not isinstance(g, dict) or 'x' not in g or 'y' not in g:
                raise RuntimeError(f'route[{i}]에 x·y가 필요: {g!r}')
        data.setdefault('frame_id', 'map')
        if data['frame_id'] != 'map':
            raise RuntimeError(f'frame_id는 map만 지원: {data["frame_id"]!r}')
        data.setdefault('repeats', 1)
        data.setdefault('settle_s', 1.0)
        data.setdefault('auto_mode', True)
        data.setdefault('pause_between', False)
        data.setdefault('move_to_start', False)
        data.setdefault('return_to_start', False)
        return data

    @staticmethod
    def _goal_xy(g: dict) -> tuple[float, float, float]:
        return float(g['x']), float(g['y']), float(g.get('yaw_deg', 0.0))

    def _nav_goal(self, g: dict, why: str) -> None:
        x, y, yaw = self._goal_xy(g)
        rc = self.bus.send_goal(x, y, yaw, wait=True)
        if rc != 0:
            raise RuntimeError(f'{why}: ({x:.2f}, {y:.2f}) 주행 실패')

    def _route(self, plan: dict, why: str) -> None:
        for i, g in enumerate(plan['route'], 1):
            self._nav_goal(g, f'{why} route {i}/{len(plan["route"])}')
            if plan['settle_s'] > 0:
                time.sleep(float(plan['settle_s']))

    def guide(self, label: str | None = None, config: str | None = None) -> int:
        """command.json의 route를 A(RL ON) → B(inner)로 반복 주행하며 rosbag 기록."""
        label = (label or time.strftime('%Y%m%d_%H%M%S')).strip()
        try:
            plan = self.load_plan(config)
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return 2
        if plan['auto_mode'] and not self.bus.request('control', 'NAV_PLAN'):
            print('FAIL: NAV_PLAN 전환 실패 — 상태/베이스 확인', file=sys.stderr)
            return 1
        start = plan.get('start')
        bags: list[str] = []
        ok_all = True
        try:
            if start and plan['move_to_start']:
                self._nav_goal(start, '시작 자세')
            ok, msg = self.toggle(True)
            if not ok:
                print(f'FAIL: RL ON 실패: {msg}', file=sys.stderr)
                return 1
            for r in range(1, int(plan['repeats']) + 1):
                if start and plan['move_to_start']:
                    self._nav_goal(start, f'{r}회차 시작 자세')
                rec = self.start(f'{label}_r{r}_A')
                bags.append(rec['bag'])
                print(f'A{r} RL 녹화: {rec["bag"]}')
                try:
                    self._route(plan, f'A{r}')
                except RuntimeError as exc:
                    print(f'FAIL: {exc}', file=sys.stderr)
                    ok_all = False
                finally:
                    self.stop()

                if start and plan['return_to_start']:
                    try:
                        self._nav_goal(start, f'A{r} → 시작 자세')
                    except RuntimeError as exc:
                        print(f'FAIL: {exc}', file=sys.stderr)
                        ok_all = False

                if plan.get('pause_between'):
                    if sys.stdin.isatty():
                        pos = (f'시작 위치 ({start["x"]:.2f}, {start["y"]:.2f}, '
                               f'{start.get("yaw_deg", 0.0):.0f}°)' if start else '시작 위치')
                        body = [f'A{r} (RL) 녹화 완료.',
                                f'로봇을 {pos}에 맞추세요.',
                                '필요하면 RViz 2D Pose Estimate / ./run-cli initpose로 위치를 보정하세요.',
                                '준비되면 Y — B(inner)를 시작합니다.']
                        if not self.bus.ui.confirm('B(inner) 준비', body=body, default=True):
                            raise RuntimeError('사용자가 B 단계를 취소')
                    else:
                        print('pause_between: TTY가 아니라 B를 바로 시작합니다')

                ok, msg = self.toggle(False)
                if not ok:
                    raise RuntimeError(f'inner controller 전환 실패: {msg}')
                rec = self.start(f'{label}_r{r}_B')
                bags.append(rec['bag'])
                print(f'B{r} inner 녹화: {rec["bag"]}')
                try:
                    self._route(plan, f'B{r}')
                except RuntimeError as exc:
                    print(f'FAIL: {exc}', file=sys.stderr)
                    ok_all = False
                finally:
                    self.stop()

                ok, msg = self.toggle(True)
                if not ok:
                    raise RuntimeError(f'RL 복귀 실패: {msg}')
                if not ok_all:
                    break
        except (RuntimeError, OSError) as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            ok_all = False
        finally:
            if self.current():
                self.stop()
        print(f'녹화 {len(bags)}개: ' + (', '.join(bags) if bags else '없음'))
        return 0 if ok_all else 1

    # --------------------------------------------------------------- CLI
    def report(self, label: str | None = None, out: str | None = None) -> int:
        """A/B bag 두 개를 HTML 보고서로 — `ab ... report --label ab1`."""
        hist = self.history()
        if label:
            hist = [h for h in hist if str(h.get('label', '')).startswith(label)]
        a = next((h for h in reversed(hist) if str(h.get('label', '')).endswith('_A')), None)
        b = next((h for h in reversed(hist) if str(h.get('label', '')).endswith('_B')), None)
        if not a or not b:
            print('A/B bag을 찾지 못함 — 먼저 guide 또는 record로 녹화하세요', file=sys.stderr)
            return 1
        try:
            from .report import generate_report
            path = generate_report(Path(a['bag']), Path(b['bag']), label=label,
                                   out=Path(out) if out else None)
        except Exception as exc:
            print(f'FAIL: 보고서 생성 — {exc}', file=sys.stderr)
            return 1
        print(f'보고서: {path}')
        return 0

    def cmd(self, args) -> int:
        target = getattr(args, 'node', None) or 'nodes'
        action = getattr(args, 'action', None)
        # 예전 `ab status`도 받아 준다 — 새 문법은 `ab rl_nav_controller status`.
        if target in LEGACY_ACTIONS and action is None:
            print(f'WARN: `ab {target}`는 예전 문법 — `ab {NODE} {target}`로 바뀌었습니다',
                  file=sys.stderr)
            action, target = target, NODE
        if target in ('nodes', 'list'):
            for name, desc in AB_NODES:
                print(f'  {name:<24} {desc}')
            print('\n사용: ab <노드> <동작> — 예: ab rl_nav_controller status')
            return 0
        if target != NODE:
            print(f'FAIL: A/B 미구현 노드: {target} (가능: {", ".join(n for n, _ in AB_NODES)})',
                  file=sys.stderr)
            return 2
        action = action or 'status'
        if action == 'status':
            self.bus._poll(1.0)   # 노드/파라미터 서비스 발견 시간
            print('\n'.join(self.status_lines()))
            return 0
        if action in ('on', 'off'):
            ok, msg = self.toggle(action == 'on')
            print(('ok: ' if ok else 'FAIL: ') + msg)
            return 0 if ok else 1
        if action == 'record':
            try:
                rec = self.start(getattr(args, 'label', None))
            except RuntimeError as exc:
                print(f'FAIL: {exc}', file=sys.stderr)
                return 1
            print(f'녹화 시작: {rec["bag"]}')
            return 0
        if action == 'stop':
            rec = self.stop()
            if rec is None:
                print('녹화 중 아님')
                return 0
            dur = max(0.0, rec['stopped'] - rec['started'])
            print(f'녹화 종료: {rec["bag"]} ({dur:.1f}s)')
            return 0
        if action == 'guide':
            return self.guide(getattr(args, 'label', None), getattr(args, 'config', None))
        if action == 'report':
            return self.report(getattr(args, 'label', None), getattr(args, 'out', None))
        if action == 'history':
            hist = self.history()
            if not hist:
                print('녹화 없음')
            for h in hist:
                print(f'{h.get("label", "?"):<12} {h["bag"]}')
            return 0
        print(f'모르는 동작: {action} (status|on|off|record|stop|guide|history)', file=sys.stderr)
        return 2
