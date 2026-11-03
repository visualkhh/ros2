"""URDF → USD만 생성 (Isaac 앱 headless, 장면은 안 띄움).

실행:
  cd isaac && $ISAACLAB_DIR/isaaclab.sh -p urdf/convert_urdf.py [robot]    # robot = urdf/ 아래 폴더 이름 (기본 beebot)

출력: isaac/urdf/<robot>/usd/<robot>/<robot>.usda (기본 beebot)
  (학습 · calibration · learn도 시작할 때 같은 함수를 자동 호출한다)
"""
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[2]          # ros2/
sys.path.insert(0, str(WS / "isaac"))

from isaaclab.app import AppLauncher  # noqa: E402

robot = sys.argv[1] if len(sys.argv) > 1 else None
sys.argv = sys.argv[:1]   # AppLauncher가 인자를 안 보게
app_launcher = AppLauncher(headless=True)

from urdf.robot_spec import robot_usd  # noqa: E402

if __name__ == "__main__":
    path = robot_usd(robot) if robot else robot_usd()
    print(f"[usd] {path}")
    app_launcher.app.close()
