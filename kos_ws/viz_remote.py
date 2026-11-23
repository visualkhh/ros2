#!/usr/bin/env python3
"""실기(Pi) kros에 붙는 Viz 실행기. 시뮬레이터/서버는 켜지 않고 화면만 띄운다.

사용:
  ./kos_ws/viz_remote.py                         # pak.local:9000
  ./kos_ws/viz_remote.py 10.2.150.172 9000
  ./kos_ws/viz_remote.py 10.2.150.172 9000 /path/to/turtlebot3/description

  python3 kos_ws/viz_remote.py ...               # Panda3D가 없는 python이면 kros/.venv로 다시 실행한다

Viz 창을 클릭해 포커스를 준 뒤:
  W/A/S/D (또는 화살표) = 텔레옵 (누르는 동안, 떼면 정지)
  마우스 왼쪽 드래그 = 화면 회전, 오른쪽 드래그 = 팬, 휠 = 줌
  start SLAM / save map / load / delete = SLAM·지도 (Pi에서 실행)
"""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# 하이브리드 노트북에서 NVIDIA GPU로 렌더링 (OpenGL 로드 전에 설정)
if Path("/proc/driver/nvidia/version").exists():
    os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
    os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")


def _ensure_panda() -> None:
    """Panda3D가 없는 python으로 실행했으면 kros/.venv의 python으로 다시 실행한다."""
    try:
        import panda3d  # noqa: F401
    except ImportError:
        venv_dir = ROOT / "kros" / ".venv"
        venv = venv_dir / "bin" / "python"
        if venv.exists() and Path(sys.prefix).resolve() != venv_dir.resolve():
            os.execv(str(venv), [str(venv), str(Path(__file__).resolve()), *sys.argv[1:]])
        raise SystemExit(
            "Panda3D가 없다. ./kos_ws/viz_remote.py 로 실행하거나 kros/.venv의 python을 써라.\n"
            f"  {ROOT}/kros/.venv/bin/python {Path(__file__).resolve()}"
        )


_ensure_panda()

from panda3d.core import WindowProperties  # noqa: E402

from kros.viz import Viz  # noqa: E402

host = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("KOS_VIZ_HOST", "pak.local")
port = int(sys.argv[2]) if len(sys.argv) > 2 else int(os.environ.get("KOS_VIZ_PORT", "9000"))
package_root = Path(sys.argv[3]).expanduser() if len(sys.argv) > 3 else ROOT / "kos_ws" / "robot" / "description"

print(f"[kos viz] {host}:{port} 에 연결 (서버가 켜져 있어야 한다)", flush=True)
print("[kos viz] 창을 클릭해 포커스를 준 뒤 W/A/S/D 로 텔레옵, 떼면 정지 / ESC 또는 Ctrl+C 로 종료", flush=True)
viz = Viz(servers=[{"ip": host, "port": port}], package_root=package_root)
props = WindowProperties()
props.setTitle(f"kros viz - {host}:{port}")
props.setSize(1000, 750)
viz.win.requestProperties(props)
try:
    viz.base.run()
finally:
    viz.close()
