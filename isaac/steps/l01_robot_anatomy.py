"""L1 — 로봇 해부: Stage · Prim · Articulation · Joint.

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./steps/run.sh l01_robot_anatomy.py

키:
  1  /World/Robot 아래 USD prim 트리 (경로·타입·물리 API) 출력
  2  joint 위치·속도, base pose, sim 시간 출력
  3  왼쪽 바퀴만 +4 rad/s (1초)   → 몸체가 오른쪽으로 돎
  4  오른쪽 바퀴만 +4 rad/s (1초) → 몸체가 왼쪽으로 돎
  5  양쪽 +4 rad/s (1초)          → 앞으로 감
  0  정지
"""
import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent          # isaac/learn
ISAAC = HERE.parent                             # isaac
sys.path.insert(0, str(ISAAC / "steps"))
sys.path.insert(0, str(ISAAC))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="L1: 로봇 해부")
parser.add_argument("--room", type=float, default=4.0, help="방 한 변 [m] (0 = 벽 없이)")
parser.add_argument("--seconds", type=float, default=0.0, help="0 = 창을 닫을 때까지")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb  # noqa: E402
import omni  # noqa: E402
import torch  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from scene import build_world, set_camera, settle, spawn_robot  # noqa: E402

DT = 1 / 120.0
FRAME_DT = 1 / 60.0


class Keyboard:
    """지금 눌린 키 집합 (이벤트 대신 상태 조회)."""

    DOWN = 1 << 3
    MAP = {f"{d}": getattr(carb.input.KeyboardInput, f"KEY_{d}") for d in "012345"}

    def __init__(self):
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()

    @property
    def pressed(self) -> set[str]:
        flags = self._input.get_keyboard_button_flags
        return {name for name, key in self.MAP.items() if flags(self._keyboard, key) & self.DOWN}


def print_tree(sim):
    stage = sim.stage
    base = "/World/Robot"
    print(f"\n[l1] USD prim tree — {base}")
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path != base and not path.startswith(base + "/"):
            continue
        depth = path.count("/") - base.count("/")
        apis = [label for label, cls in (
            ("RigidBody", UsdPhysics.RigidBodyAPI),
            ("Collision", UsdPhysics.CollisionAPI),
            ("Mass", UsdPhysics.MassAPI),
            ("Articulation", UsdPhysics.ArticulationRootAPI),
        ) if prim.HasAPI(cls)]
        print("  " * depth + f"{prim.GetName()}  [{prim.GetTypeName()}]  {' '.join(apis)}")


def print_state(robot, sim_t):
    jp = robot.data.joint_pos.torch[0].tolist()
    jv = robot.data.joint_vel.torch[0].tolist()
    print("\n[l1] joints")
    for name, p, v in zip(robot.joint_names, jp, jv):
        print(f"  {name:20s} pos={p:+.3f} rad  vel={v:+.3f} rad/s")
    x, y, z = robot.data.root_pos_w.torch[0].tolist()
    yaw = robot.data.heading_w.torch[0].item()
    print(f"[l1] base x={x:+.3f} y={y:+.3f} z={z:+.3f} m · yaw={yaw:+.3f} rad")
    print(f"[l1] sim time {sim_t:.2f} s · physics dt {DT:.5f} s · links {len(robot.body_names)}")


def main():
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))
    set_camera(sim)
    build_world(1 if args_cli.room > 0 else 0, args_cli.room, 0)
    scene, robot, _ = spawn_robot()
    sim.reset()
    z = settle(scene, robot, sim)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    dev = robot.device
    print(f"[l1] base 높이 {z:.3f} m · Articulation: joint {len(robot.joint_names)}개 · link {len(robot.body_names)}개")
    print("[l1] 1=prim tree · 2=joint/pose · 3=왼쪽 바퀴 · 4=오른쪽 바퀴 · 5=양쪽 · 0=정지")
    print_tree(sim)

    kb = Keyboard()
    target = (0.0, 0.0)
    action_end = 0.0
    sim_t = 0.0
    prev_keys = set()
    t0 = time.perf_counter()

    while simulation_app.is_running():
        frame_start = time.perf_counter()
        wall = frame_start - t0
        if args_cli.seconds > 0 and wall > args_cli.seconds:
            break

        keys = kb.pressed
        new = keys - prev_keys
        prev_keys = keys
        if "1" in new:
            print_tree(sim)
        if "2" in new:
            print_state(robot, sim_t)
        if "3" in new:
            target, action_end = (4.0, 0.0), sim_t + 1.0
            print("[l1] 왼쪽 바퀴 +4 rad/s (1초)")
        if "4" in new:
            target, action_end = (0.0, 4.0), sim_t + 1.0
            print("[l1] 오른쪽 바퀴 +4 rad/s (1초)")
        if "5" in new:
            target, action_end = (4.0, 4.0), sim_t + 1.0
            print("[l1] 양쪽 +4 rad/s (1초)")
        if "0" in new:
            target, action_end = (0.0, 0.0), 0.0
            print("[l1] 정지")
        if target != (0.0, 0.0) and sim_t >= action_end:
            target = (0.0, 0.0)

        robot.set_joint_velocity_target(torch.tensor([target], device=dev), joint_ids=wheel_ids)
        scene.write_data_to_sim()
        sim.step(render=False)
        sim.step(render=False)
        scene.update(2 * DT)
        sim.render()
        sim_t += 2 * DT

        rest = FRAME_DT - (time.perf_counter() - frame_start)
        if rest > 0:
            time.sleep(rest)

    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
