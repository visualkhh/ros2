"""런타임 값 저장소 — ROS 없이 즉시 실행 (원격 SSH에서 CLI 기동 없이 쓰기).

  python3 op/runtime_state.py set controller_server FollowPath.enable false
  python3 op/runtime_state.py clear controller_server FollowPath.enable
  python3 op/runtime_state.py status
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

RUN_DIR = Path(__file__).resolve().parents[2] / '.run'   # tools/.run
PATH = RUN_DIR / 'runtime_params.json'


def load() -> dict:
    try:
        data = json.loads(PATH.read_text())
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save(data: dict) -> None:
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    tmp = PATH.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(PATH)


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__.strip())
        return 2
    cmd = argv[0]
    data = load()
    if cmd == 'set' and len(argv) == 4:
        node, name, raw = argv[1:]
        value = {'true': True, 'false': False}.get(raw.lower(), raw)
        data.setdefault(node, {})[name] = value
        save(data)
        print(f'런타임 값 저장: {node} {name}={value}')
        return 0
    if cmd == 'clear' and len(argv) == 3:
        node, name = argv[1:]
        data.get(node, {}).pop(name, None)
        if node in data and not data[node]:
            data.pop(node)
        save(data)
        print(f'런타임 값 삭제: {node} {name}')
        return 0
    if cmd == 'status':
        print(json.dumps(data, ensure_ascii=False, indent=2))
        return 0
    print(__doc__.strip(), file=sys.stderr)
    return 2


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
