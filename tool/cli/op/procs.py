"""Procs — tool이 직접 띄우는 launch 프로세스 관리 (start/stop/상태/로그).

state_manager 같은 상주 노드가 없으므로 tool이 `ros2 launch`를 자기 세션(setsid)으로
띄우고 pid를 tool/.run/<이름>.pid 에 적는다. 끌 때는 프로세스 그룹 전체에
SIGINT → (10초) SIGTERM → (3초) SIGKILL. 로그는 tool/.run/<이름>.log.

CLI를 껐다 켜도 pid 파일로 다시 찾으므로 SLAM·Nav는 계속 돈다.
"""
from __future__ import annotations

import os
import shlex
import socket
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

TOOL_DIR = Path(__file__).resolve().parents[2]          # tool/
REPO_DIR = TOOL_DIR.parent                               # ros2/
WS_DIR = REPO_DIR / 'ros2_ws'
CONFIG_DIR = TOOL_DIR / 'config'
RUN_DIR = TOOL_DIR / '.run'
MAP_DIR = Path(os.environ.get('TOOL_MAP_DIR', WS_DIR / 'map'))

SLAM_PARAMS = Path(os.environ.get('TOOL_SLAM_PARAMS', CONFIG_DIR / 'slam_toolbox.yaml'))
NAV_PARAMS = Path(os.environ.get('TOOL_NAV_PARAMS', CONFIG_DIR / 'nav2_params.yaml'))
# 베이스(바퀴·odom·라이다) bringup. 기본 = 로봇만(RL 없음).
#   RL on으로 시작: TOOL_BASE_CMD='ros2 launch tb3_bringup tb-only.launch.py rl:=true' (보통은 ./run-cli rl on)
BASE_CMD = os.environ.get('TOOL_BASE_CMD', 'ros2 launch tb3_bringup tb-only.launch.py')
SIM = os.environ.get('TOOL_USE_SIM_TIME', 'false')
# 베이스·SLAM·Nav 프로세스와 지도 파일은 로봇(Pi)에 있어야 한다. 로봇이 아닌 곳에서 run-cli를 쓰면
# RemoteProcs가 켜기/끄기·지도 저장/목록·로그를 SSH로 로봇의 run-cli에 넘기고, 켜짐 여부는 ROS 노드 그래프로 본다
# (그래서 토픽이 보이는 곳 어디서나 같은 CLI). PC 시뮬레이션(로컬에 띄우기)은 TOOL_LOCAL=1.
ROBOT_HOST = os.environ.get('ROBOT_HOST', 'pak.local')
ROBOT_TOOL_DIR = os.environ.get('TOOL_ROBOT_DIR', '~/ros2/tool')   # 로봇의 tool 위치
ROBOT_SSH = os.environ.get('TOOL_ROBOT_SSH', f'pak@{ROBOT_HOST}')   # SSH 대상 (키 인증 필요)
IS_ROBOT = socket.gethostname() == ROBOT_HOST.removesuffix('.local') or os.environ.get('TOOL_LOCAL') == '1'


@dataclass
class Service:
    name: str
    label: str
    desc: str
    conflicts: tuple[str, ...] = ()

    def command(self, map_yaml: str | None = None) -> list[str]:
        if self.name == 'base':
            return shlex.split(BASE_CMD)
        if self.name == 'slam':
            return ['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                    f'slam_params_file:={SLAM_PARAMS}', f'use_sim_time:={SIM}']
        if self.name == 'nav':
            if not map_yaml:
                raise ValueError('nav는 지도(map.yaml)가 필요합니다')
            return ['ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                    f'map:={map_yaml}', f'params_file:={NAV_PARAMS}',
                    f'use_sim_time:={SIM}', 'autostart:=true']
        if self.name == 'explore':
            # SLAM이 /map·map→odom을 내는 중에 Nav2 주행만 (지도 없이, amcl 없이)
            return ['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                    f'params_file:={NAV_PARAMS}', f'use_sim_time:={SIM}', 'autostart:=true']
        raise ValueError(self.name)


SERVICES: dict[str, Service] = {s.name: s for s in (
    Service('base', '베이스', '바퀴·odom·라이다 bringup (TOOL_BASE_CMD)'),
    Service('slam', 'SLAM 맵핑', 'slam_toolbox online_async — /map 생성', ('nav',)),
    Service('nav', 'Nav (지도 주행)', 'nav2 bringup — map_server + amcl + 주행', ('slam', 'explore')),
    Service('explore', 'SLAM 중 Nav', 'nav2 navigation만 — SLAM 지도 위 목표 주행', ('nav',)),
)}


class Procs:
    def __init__(self):
        RUN_DIR.mkdir(parents=True, exist_ok=True)
        self.map_dir_txt = str(MAP_DIR)   # 화면 표시용 (원격이면 로봇:경로)

    # ------------------------------------------------------------ pid 파일
    @staticmethod
    def _pidfile(name: str) -> Path:
        return RUN_DIR / f'{name}.pid'

    @staticmethod
    def logfile(name: str) -> Path:
        return RUN_DIR / f'{name}.log'

    def pid(self, name: str) -> int | None:
        """살아 있는 launch pid · 없거나 죽었으면 None (죽은 pid 파일은 지운다)."""
        f = self._pidfile(name)
        try:
            pid = int(f.read_text().split()[0])
        except (OSError, ValueError, IndexError):
            return None
        try:
            os.kill(pid, 0)
            cmd = Path(f'/proc/{pid}/cmdline').read_bytes()
        except (OSError, ProcessLookupError):
            f.unlink(missing_ok=True)
            return None
        if b'ros2' not in cmd and b'launch' not in cmd:
            f.unlink(missing_ok=True)  # pid 재사용 — 다른 프로세스
            return None
        return pid

    def running(self, name: str) -> bool:
        return self.pid(name) is not None

    def info(self, name: str) -> str:
        """pid 파일 두 번째 줄 = 시작 때 남긴 메모 (예: 지도 경로)."""
        try:
            lines = self._pidfile(name).read_text().splitlines()
            return lines[1] if len(lines) > 1 else ''
        except OSError:
            return ''

    def uptime(self, name: str) -> float | None:
        try:
            return time.time() - self._pidfile(name).stat().st_mtime if self.running(name) else None
        except OSError:
            return None

    # ------------------------------------------------------------ start/stop
    def start(self, name: str, map_yaml: str | None = None) -> tuple[bool, str]:
        svc = SERVICES[name]
        if self.running(name):
            return False, f'{svc.label}: 이미 실행 중 (pid {self.pid(name)})'
        busy = [SERVICES[c].label for c in svc.conflicts if self.running(c)]
        if busy:
            return False, f'{svc.label}: 먼저 끄세요 → {", ".join(busy)}'
        try:
            cmd = svc.command(map_yaml)
        except ValueError as exc:
            return False, f'{svc.label}: {exc}'
        log = open(self.logfile(name), 'w')
        log.write(f'$ {shlex.join(cmd)}\n')
        log.flush()
        p = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True, cwd=str(WS_DIR if WS_DIR.is_dir() else TOOL_DIR))
        self._pidfile(name).write_text(f'{p.pid}\n{map_yaml or ""}\n')
        time.sleep(1.5)  # 바로 죽는 경우(패키지 없음·파라미터 오류) 잡기
        if p.poll() is not None:
            self._pidfile(name).unlink(missing_ok=True)
            return False, f'{svc.label}: 시작 직후 종료 (code {p.returncode}) — 로그: {self.logfile(name)}\n' \
                          + self.tail(name, 15)
        return True, f'{svc.label}: 시작 (pid {p.pid}) · 로그 {self.logfile(name)}'

    def stop(self, name: str, timeout: float = 10.0) -> tuple[bool, str]:
        svc = SERVICES[name]
        pid = self.pid(name)
        if pid is None:
            return True, f'{svc.label}: 실행 중 아님'
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0), (signal.SIGKILL, 1.0)):
            try:
                os.killpg(pid, sig)
            except ProcessLookupError:
                break
            end = time.monotonic() + wait
            while time.monotonic() < end and self._group_alive(pid):
                time.sleep(0.2)
            if not self._group_alive(pid):
                break
        self._pidfile(name).unlink(missing_ok=True)
        return True, f'{svc.label}: 종료 (pid {pid})'

    def restart(self, name: str) -> tuple[bool, str]:
        info = self.info(name) or None   # nav: 같은 지도로
        _, msg = self.stop(name)
        ok, msg2 = self.start(name, info)
        return ok, f'{msg}\n{msg2}'

    @staticmethod
    def _group_alive(pgid: int) -> bool:
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def tail(self, name: str, n: int = 40) -> str:
        try:
            lines = self.logfile(name).read_text(errors='replace').splitlines()
        except OSError:
            return '(로그 없음)'
        return '\n'.join(lines[-n:])

    # ------------------------------------------------------------ 지도 파일
    @staticmethod
    def maps() -> list[tuple[str, Path]]:
        """MAP_DIR/<이름>/map.yaml 목록 (최신 먼저)."""
        if not MAP_DIR.is_dir():
            return []
        found = [(d.name, d / 'map.yaml') for d in MAP_DIR.iterdir() if (d / 'map.yaml').is_file()]
        return sorted(found, key=lambda x: x[1].stat().st_mtime, reverse=True)

    @staticmethod
    def read_map(path) -> str:
        return Path(path).read_text(errors='replace')

    @staticmethod
    def resolve_map(name_or_path: str | None) -> Path | None:
        """지도 이름·폴더·yaml 경로 → map.yaml. None이면 가장 최근 지도."""
        if not name_or_path:
            maps = Procs.maps()
            return maps[0][1] if maps else None
        p = Path(name_or_path).expanduser()
        for cand in (p, p / 'map.yaml', MAP_DIR / name_or_path / 'map.yaml'):
            if cand.is_file() and cand.suffix == '.yaml':
                return cand.resolve()
        return None

    @staticmethod
    def save_map(name: str) -> tuple[bool, str]:
        """지금 /map을 MAP_DIR/<name>/map.{pgm,yaml}로 저장 (nav2 map_saver_cli)."""
        out = MAP_DIR / name
        out.mkdir(parents=True, exist_ok=True)
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(out / 'map'),
               '--ros-args', '-p', 'save_map_timeout:=10.0']
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=40)
        except subprocess.TimeoutExpired:
            return False, '시간 초과 — /map이 발행되고 있는지 확인 (SLAM 실행 중?)'
        text = (r.stdout + r.stderr).strip()
        ok = r.returncode == 0 and (out / 'map.yaml').is_file()
        if not ok and not any(out.iterdir()):
            out.rmdir()
        return ok, (f'저장: {out}/map.yaml · map.pgm' if ok else '저장 실패') + '\n' + text[-1500:]


# 켜짐 판단에 쓰는 노드 (ROS 그래프 — 연결된 곳 어디서나 보임)
MARKER_NODES = {'base': ('opencr_driver',), 'slam': ('slam_toolbox',),
                'nav': ('amcl',), 'explore': ('bt_navigator',)}


class RemoteProcs(Procs):
    """로봇이 아닌 곳: 켜기/끄기·지도·로그는 SSH로 로봇의 run-cli에, 켜짐 여부는 노드 그래프로."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.host = ROBOT_HOST
        self.map_dir_txt = f'{ROBOT_HOST}:{ROBOT_TOOL_DIR}/../ros2_ws/map'
        self._maps: tuple[float, list] = (0.0, [])

    def _ssh(self, remote_cmd: str, timeout: float = 120) -> tuple[int, str]:
        cmd = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', ROBOT_SSH, remote_cmd]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return 124, f'{self.host}: 시간 초과 ({timeout:.0f}s)'
        out = '\n'.join(ln for ln in (r.stdout + r.stderr).splitlines()
                        if not ln.startswith(('ROS_DOMAIN_ID=', 'DDS 피어', 'ROS_STATIC_PEERS')))
        if r.returncode == 255:
            out += f'\n(SSH 실패 — `ssh {ROBOT_SSH}`가 비번 없이 되는지 확인: documents/ai/ai-passwordless-robot-access.md)'
        return r.returncode, out.strip()

    def _cli(self, *args: str, timeout: float = 120) -> tuple[int, str]:
        return self._ssh(f'cd {ROBOT_TOOL_DIR} && ./run-cli {shlex.join(args)}', timeout)

    def running(self, name: str) -> bool:
        names = {n for n, _ in self.node.get_node_names_and_namespaces()}
        if name == 'explore':   # SLAM 중 Nav = 주행부는 있는데 amcl은 없음
            return 'bt_navigator' in names and 'amcl' not in names
        return any(m in names for m in MARKER_NODES[name])

    def pid(self, name: str) -> int | None:
        return None

    def uptime(self, name: str) -> float | None:
        return None

    def info(self, name: str) -> str:
        return ''

    def logfile(self, name: str) -> str:
        return f'{self.host}:{ROBOT_TOOL_DIR}/.run/{name}.log'

    def start(self, name: str, map_yaml: str | None = None) -> tuple[bool, str]:
        rc, out = self._cli(name, 'start', *([map_yaml] if map_yaml else []))
        return rc == 0, f'[{self.host}] {out}'

    def stop(self, name: str, timeout: float = 10.0) -> tuple[bool, str]:
        rc, out = self._cli(name, 'stop')
        return rc == 0, f'[{self.host}] {out}'

    def restart(self, name: str) -> tuple[bool, str]:
        rc, out = self._cli(name, 'restart')
        return rc == 0, f'[{self.host}] {out}'

    def tail(self, name: str, n: int = 40) -> str:
        return self._ssh(f'tail -n {int(n)} {ROBOT_TOOL_DIR}/.run/{name}.log 2>&1', 15)[1]

    def maps(self) -> list[tuple[str, PurePosixPath]]:
        # 메뉴가 2초마다 다시 그리므로 5초 캐시 (SSH 왕복)
        if time.monotonic() - self._maps[0] < 5.0:
            return self._maps[1]
        rc, out = self._ssh(f'ls -1t {ROBOT_TOOL_DIR}/../ros2_ws/map/*/map.yaml 2>/dev/null', 15)
        paths = [PurePosixPath(ln) for ln in out.splitlines() if ln.endswith('/map.yaml')]
        found = [(p.parent.name, p) for p in paths]
        self._maps = (time.monotonic(), found)
        return found

    def read_map(self, path) -> str:
        return self._ssh(f'cat {shlex.quote(str(path))}', 15)[1]

    def resolve_map(self, name_or_path: str | None):
        maps = self.maps()
        if not name_or_path:
            return maps[0][1] if maps else None
        return next((p for n, p in maps if name_or_path in (n, str(p))), None)

    def save_map(self, name: str) -> tuple[bool, str]:
        self._maps = (0.0, [])
        rc, out = self._cli('map', 'save', name)
        return rc == 0, f'[{self.host}] {out}'


def make_procs(node) -> Procs:
    return Procs() if IS_ROBOT else RemoteProcs(node)
