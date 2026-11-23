"""TurtleBot3 예제 실행 (IDE 디버그용). `python -m kros.examples.turtlebot3` 또는 이 파일을 직접 실행.

`python -m kros.examples`와 같다. 파일로 직접 실행할 때 저장소 루트(ros2/)가 경로에 없으면 넣어 준다.
"""
import os
import sys
from pathlib import Path

if not __package__:  # 파일로 직접 실행
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

if Path("/proc/driver/nvidia/version").exists():
    os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
    os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")

from kros.examples.turtlebot3.main import main  # noqa: E402

main()
