"""S1 — 우리 로봇을 Isaac Sim 창에 띄운다 (방 + 조명 + 카메라).

목적: URDF → USD 변환이 우리 로봇으로 보이는지, 바닥에 제대로 서는지 눈으로 확인.
실행 (Isaac Lab venv):
  cd /home/pank/pa-course/IsaacLab-6.1
  ./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/steps/s01_robot_in_world.py --viz kit

  # 또는 래퍼 (창 옵션 자동):
  /home/pank/workspaces/visualkhh/source/ros2/isaac/steps/run.sh s01_robot_in_world.py

창이 뜨면 뷰포트에서 마우스로 돌려 보고(휠 = 줌), 다 보면 창을 닫는다.
콘솔의 [s1] 줄이 통과 조건: base 높이 ≈ 0.01 m (넘어지지 않음).
(이 Isaac Lab 버전은 --headless 대신 --viz none / --viz kit 을 쓴다.)
"""
import argparse
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parents[2]          # ros2/
ISAAC = WS / "isaac"
sys.path.insert(0, str(ISAAC))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="S1: 우리 로봇 + 방을 창에 띄운다")
parser.add_argument("--room", type=float, default=4.0, help="방 한 변 길이 [m] (0 = 벽 없이)")
parser.add_argument("--seconds", type=float, default=0.0, help="0 = 창을 닫을 때까지")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from tasks.common.robot_spec import robot_usd  # noqa: E402

WALL_H = 1.1
WALL_T = 0.05


def wall(path: str, size, pos):
    cfg = sim_utils.CuboidCfg(
        size=size,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.72, 0.72, 0.70)),
    )
    cfg.func(path, cfg, translation=pos)


def box(path: str, size, pos, color):
    cfg = sim_utils.CuboidCfg(size=size, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color))
    cfg.func(path, cfg, translation=pos)


def build_room(size: float):
    """바닥·조명·벽·기준 물체. size=0이면 빈 바닥만."""
    spawn_ground_plane("/World/ground", GroundPlaneCfg())
    light = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.95, 0.95, 0.95))
    light.func("/World/light", light, translation=(2.0, -2.0, 4.0))
    if size <= 0:
        return
    half = size / 2.0
    # 벽 4개 (위는 열림 — 카메라가 내려다봄)
    wall("/World/room/wall_px", (WALL_T, size, WALL_H), (half, 0.0, WALL_H / 2))
    wall("/World/room/wall_nx", (WALL_T, size, WALL_H), (-half, 0.0, WALL_H / 2))
    wall("/World/room/wall_py", (size, WALL_T, WALL_H), (0.0, half, WALL_H / 2))
    wall("/World/room/wall_ny", (size, WALL_T, WALL_H), (0.0, -half, WALL_H / 2))
    # 크기 감각용 물체 3개 (색으로 구분 — 나중에 랜덤 배치의 씨앗)
    box("/World/room/box_red", (0.3, 0.3, 0.3), (0.9, 0.7, 0.15), (0.75, 0.15, 0.12))
    box("/World/room/box_green", (0.5, 0.25, 0.8), (-1.1, 0.9, 0.4), (0.12, 0.55, 0.20))
    box("/World/room/box_blue", (0.2, 0.6, 0.2), (1.2, -1.0, 0.1), (0.15, 0.30, 0.75))


def main():
    usd_path = robot_usd()
    print(f"[s1] robot USD -> {usd_path}")

    sim = SimulationContext(SimulationCfg(dt=1 / 120, device=args_cli.device))
    sim.set_camera_view(eye=(1.8, -1.8, 1.55), target=(0.0, 0.0, 0.15))
    build_room(args_cli.room)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        num_envs = 1
        env_spacing = 2.0

    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.03)),
        actuators={"wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*wheel.*"], effort_limit=2.0,
            velocity_limit=8.0, stiffness=0.0, damping=40.0)},
    )
    scene = InteractiveScene(SceneCfg())
    robot = Articulation(robot_cfg)
    scene.articulations["robot"] = robot
    sim.reset()

    # 1초 정착 (0.03 m에서 떨어져 바퀴·캐스터로 섬)
    dt = sim.get_physics_dt()
    for _ in range(120):
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
    z = robot.data.root_pos_w.torch[0, 2].item()
    print("[s1] joints:", ", ".join(robot.joint_names))
    print("[s1] links :", ", ".join(robot.body_names))
    print(f"[s1] base 높이: {z:.3f} m (기대 ≈ 0.01 — 0.00이면 넘어짐/스폰 실패)")
    print("[s1] 창 조작: 마우스 오른쪽 드래그 = 회전, 가운데 = 이동, 휠 = 줌, F = 선택 물체로 확대")

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
