"""URDF → USD만 생성 (Isaac 앱 headless, 장면은 안 띄움).

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./convert_usd.sh

출력: isaac/usd/tb3_waffle_pi/tb3_waffle_pi.usda
  (S1/L1/S2/S5 스크립트도 시작할 때 같은 함수를 자동 호출한다)
"""
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[2]          # ros2/
sys.path.insert(0, str(WS / "isaac"))

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher(headless=True)

from tasks.common.robot_spec import robot_usd  # noqa: E402

if __name__ == "__main__":
    path = robot_usd()
    print(f"[usd] {path}")
    app_launcher.app.close()
