"""Viz-only runner: connects to a remote kros server and displays Viz.

사용법:
  python viz_remote.py [server_ip] [server_port] [package_root]

예:
  python viz_remote.py 192.168.1.10 9000
  python viz_remote.py 127.0.0.1 9000 /path/to/turtlebot3/description

서버(main.py)가 먼저 실행되어 있어야 한다.
"""
import os
import sys
from pathlib import Path

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # ros2/

if Path("/proc/driver/nvidia/version").exists():
    os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
    os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")

from kros import Kros
from kros.viz import Viz
from direct.showbase.ShowBase import ShowBase

server_ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
server_port = int(sys.argv[2]) if len(sys.argv) > 2 else 9000

kros = None
base = None
viz = Viz(servers=[{"ip": server_ip, "port": server_port}])
print(f"[viz_remote] connected to {server_ip}:{server_port}", flush=True)
viz.base.run()