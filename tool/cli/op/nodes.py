"""개별 ROS 노드 중지·재시작 (OS 프로세스 단위 — 프로세스가 있는 로봇에서만 실행).

ros2 launch는 Node마다 별도 OS 프로세스로 띄우며 argv에 `-r __node:=<이름>` 토큰이
들어간다. 그 토큰으로 PID를 찾아 kill하고, 재시작은 kill 전 argv 그대로 다시 실행한다.
launch 파일에 respawn이 없어서(전부 없음) kill한 노드는 그냥 죽은 채로 남는다.
재시작한 노드는 launch의 자식이 아니라서 서비스 stop-all에 안 걸리므로
.run/nodes/<이름>.pid로 추적하고 stop-all 때 같이 정리한다.

PC에서는 RemoteProcs가 SSH로 로봇의 run-cli node-kill/node-restart에 넘긴다.
"""
from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

from .procs import RUN_DIR, WS_DIR

NODE_DIR = RUN_DIR / 'nodes'

#: 이 CLI 자신의 rclpy 노드명 — 절대 건드리지 않음
SELF = 'robot_tool'
#: 끄면 제어·주행이 멈추는 핵심 노드 — --force(메뉴는 확인) 없이 거부
CRITICAL = {'state_manager', 'cmd_manager', 'opencr_driver'}


def base(name: str) -> str:
    """'/scan_resample' → 'scan_resample' (우리 그래프는 네임스페이스 없음)."""
    return (name or '').strip().lstrip('/').split('/')[-1]


def _argv(pid: int) -> list[str] | None:
    try:
        raw = Path(f'/proc/{pid}/cmdline').read_bytes()
    except OSError:
        return None
    if not raw.strip(b'\0'):
        return None
    return [a.decode(errors='replace') for a in raw.split(b'\0') if a]


def _alive(pid: int) -> bool:
    """좀비(Z)는 죽은 것으로 본다 (launch가 회수하기 전 잔재)."""
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return False
    state = stat.rsplit(')', 1)[-1].split()
    return bool(state) and state[0] != 'Z'


def find_node_procs(name: str) -> list[tuple[int, list[str]]]:
    """argv에 `__node:=<이름>` 토큰이 있는 프로세스. 자기 자신 제외."""
    me = os.getpid()
    want = f'__node:={base(name)}'
    out = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        argv = _argv(pid)
        if not argv or want not in argv:
            continue
        if any(SELF in a for a in argv):
            continue
        out.append((pid, argv))
    return out


def _tracked_pid(name: str) -> int | None:
    """재시작 추적 파일의 pid — 살아 있고 토큰이 맞을 때만."""
    try:
        pid = int((NODE_DIR / f'{base(name)}.pid').read_text().split()[0])
    except (OSError, ValueError, IndexError):
        return None
    argv = _argv(pid)
    if argv and f'__node:={base(name)}' in argv and _alive(pid):
        return pid
    return None


def _untrack(name: str) -> None:
    (NODE_DIR / f'{base(name)}.pid').unlink(missing_ok=True)


def _kill_pid(pid: int, timeout: float = 5.0) -> bool:
    try:
        os.kill(pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        return not _alive(pid)
    end = time.monotonic() + timeout
    while time.monotonic() < end and _alive(pid):
        time.sleep(0.2)
    if _alive(pid):
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        time.sleep(0.5)
    return not _alive(pid)


def node_kill(name: str, force: bool = False) -> tuple[bool, str]:
    name = base(name)
    if not name or name in (SELF, 'run-cli'):
        return False, f'{name or "(빈 이름)"}: 자기 자신은 끌 수 없음'
    if name in CRITICAL and not force:
        return False, f'{name}: 제어 핵심 노드 — 메뉴에서 확인하거나 `node-kill {name} --force`'
    targets = {pid for pid, _ in find_node_procs(name)}
    tracked = _tracked_pid(name)
    if tracked:
        targets.add(tracked)
    if not targets:
        return False, f'{name}: 로봇에 OS 프로세스 없음 (다른 머신의 노드이거나 이미 종료)'
    dead = sorted(p for p in targets if _kill_pid(p))
    failed = sorted(targets - set(dead))
    _untrack(name)
    msg = f'{name}: 프로세스 {len(dead)}개 종료 ({", ".join(map(str, dead))})'
    if failed:
        return False, msg + f' — 실패 {failed} (권한?)'
    return True, msg


def _tail(path: Path, n: int = 15) -> str:
    try:
        lines = path.read_text(errors='replace').splitlines()
    except OSError:
        return '(로그 없음)'
    return '\n'.join(lines[-n:])


def node_restart(name: str, force: bool = False) -> tuple[bool, str]:
    """kill 후 kill 전 argv 그대로 재실행. 프로세스가 정확히 1개일 때만."""
    name = base(name)
    if not name or name in (SELF, 'run-cli'):
        return False, f'{name or "(빈 이름)"}: 자기 자신은 재시작할 수 없음'
    if name in CRITICAL and not force:
        return False, f'{name}: 제어 핵심 노드 — 메뉴에서 확인하거나 `node-restart {name} --force`'
    procs = find_node_procs(name)
    if len(procs) != 1:
        detail = f' (pid {", ".join(str(p) for p, _ in procs)})' if procs else ''
        return False, f'{name}: 재시작은 프로세스 1개일 때만 (현재 {len(procs)}개){detail}'
    pid, argv = procs[0]
    if not _kill_pid(pid):
        return False, f'{name}: 기존 프로세스(pid {pid})가 안 죽음 — 서비스 재시작으로 처리'
    _untrack(name)
    log = RUN_DIR / f'node-{name}.log'
    NODE_DIR.mkdir(parents=True, exist_ok=True)
    with open(log, 'a') as f:
        f.write(f'$ {" ".join(argv)}\n')
        f.flush()
        p = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=f, stderr=subprocess.STDOUT,
                             start_new_session=True, cwd=str(WS_DIR))
    (NODE_DIR / f'{name}.pid').write_text(f'{p.pid}\nrestarted\n')
    time.sleep(1.0)
    if p.poll() is not None:
        return False, f'{name}: 재실행 직후 종료 (code {p.returncode}) — 로그: {log}\n' + _tail(log)
    return True, f'{name}: 재시작 (pid {pid} → {p.pid}) · 로그 {log}'


def tracked_nodes() -> dict[str, int]:
    """재시작 추적 중(살아 있음)인 노드 — {이름: pid}."""
    if not NODE_DIR.is_dir():
        return {}
    out = {}
    for f in NODE_DIR.glob('*.pid'):
        pid = _tracked_pid(f.stem)
        if pid:
            out[f.stem] = pid
    return out


def cleanup_tracked() -> str:
    """stop-all용 — 재시작 추적 노드 전부 종료."""
    found = tracked_nodes()
    if not found:
        return '재시작 추적 노드 없음'
    dead = sorted(n for n, p in found.items() if _kill_pid(p))
    for n in dead:
        _untrack(n)
    left = sorted(set(found) - set(dead))
    msg = f'추적 노드 종료: {", ".join(dead)}' if dead else '추적 노드 종료 실패'
    return msg + (f' — 남은 {left}' if left else '')
