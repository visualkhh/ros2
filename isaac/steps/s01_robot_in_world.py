"""S1 — 우리 로봇을 Isaac Sim 창에 띄운다 (방 + 조명 + 카메라).

목적: URDF → USD 변환이 우리 로봇으로 보이는지, 바닥에 제대로 서는지 눈으로 확인.
실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./steps/run.sh s01_robot_in_world.py

창이 뜨면 뷰포트에서 마우스로 돌려 보고(휠 = 줌), 다 보면 창을 닫는다.
콘솔의 [s1] 줄이 통과 조건: base 높이 ≈ 0.01 m (넘어지지 않음).
(이 Isaac Lab 버전은 --headless 대신 --viz none / --viz kit 을 쓴다.)
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="S1: 우리 로봇 + 방을 창에 띄운다")
parser.add_argument("--room", type=float, default=4.0, help="방 한 변 길이 [m] (0 = 벽 없이)")
parser.add_argument("--seconds", type=float, default=0.0, help="0 = 창을 닫을 때까지")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from scene import build_room, set_camera, settle, spawn_robot  # noqa: E402


def main():
    sim = SimulationContext(SimulationCfg(dt=1 / 120, device=args_cli.device))
    set_camera(sim)
    build_room(args_cli.room)
    scene, robot, usd_path = spawn_robot()
    sim.reset()
    z = settle(scene, robot, sim)
    print(f"[s1] robot USD -> {usd_path}")
    print("[s1] joints:", ", ".join(robot.joint_names))
    print("[s1] links :", ", ".join(robot.body_names))
    print(f"[s1] base 높이: {z:.3f} m (기대 ≈ 0.01 — 0.00이면 넘어짐/스폰 실패)")
    print("[s1] 창 조작: 오른쪽 드래그 = 회전, 가운데 = 이동, 휠 = 줌, F = 선택 물체로 확대")

    # 정착 후엔 물리를 멈추고 화면만 갱신 — 마우스로 카메라 조작 가능
    deadline = time.time() + args_cli.seconds if args_cli.seconds > 0 else None
    while simulation_app.is_running():
        simulation_app.update()
        if deadline and time.time() > deadline:
            break

    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
