"""examples 실행. 둘 다 된다: `python3 -m kros.examples` (ros2/ 에서) 또는 이 파일을 직접 실행(IDE 실행 버튼). 지금은 turtlebot3 하나."""
import os
import sys
from pathlib import Path

if not __package__:  # 파일로 직접 실행하면 저장소 루트(ros2/)가 경로에 없어서 kros를 못 찾는다
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

# 하이브리드 노트북에서 NVIDIA GPU로 렌더링 (OpenGL이 로드되기 전에 설정). NVIDIA 드라이버가 없으면 건드리지 않는다.
if Path("/proc/driver/nvidia/version").exists():
    os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
    os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")

from kros.examples.turtlebot3.main import main  # noqa: E402

main()
