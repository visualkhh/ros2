"""URDF → USD만 생성 (Isaac 앱 headless, 장면은 안 띄움).

실행:
  ./run-usd.sh [robot]   (= isaaclab.sh -p robots/convert_urdf.py [robot])    # robot = robots/ 아래 폴더 이름 또는 URDF 폴더

출력: <robot 폴더>/usd/<robot>/<robot>.usda
  (학습 · calibration · learn도 시작할 때 같은 함수를 자동 호출한다)
"""
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[2]          # ros2/
sys.path.insert(0, str(WS / "isaac"))

from isaaclab.app import AppLauncher  # noqa: E402

if len(sys.argv) < 2:
    raise SystemExit("사용: ./run-usd.sh <robots/ 아래 이름 | URDF 폴더>")
robot = sys.argv[1]
sys.argv = sys.argv[:1]   # AppLauncher가 인자를 안 보게
app_launcher = AppLauncher(headless=True)

from robots.urdf_to_usd import robot_usd  # noqa: E402

if __name__ == "__main__":
    path = robot_usd(robot)
    print(f"[usd] {path}")
    app_launcher.app.close()
