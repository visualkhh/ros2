"""L2 — 물리 시간: sim time vs 벽시계, dt와 substep.

로봇은 항상 같은 명령(0.12 m/s)으로 움직인다. 바뀌는 건 "한 화면(frame)마다
물리를 몇 번 돌리느냐"(steps/frame)뿐. 화면에서 로봇이 빨라/느려 보이는 이유를 본다.

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./steps/run.sh ../learn/l02_physics_time.py

키:
  1  한 프레임에 물리 1스텝  (느리게 보임)
  2  한 프레임에 물리 2스텝  (기본)
  4  한 프레임에 물리 4스텝  (빠르게 보임)
  8  한 프레임에 물리 8스텝  (아주 빠름)
  0  정지

콘솔:
  [l2] steps/frame=2 · sim 2.00s / wall 1.00s (2.00x) · x=0.12m
  → sim time / wall time = "시뮬이 벽시계보다 몇 배 빠른가"
"""
import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parent
sys.path.insert(0, str(ISAAC / "steps"))
sys.path.insert(0, str(ISAAC))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="L2: 물리 시간")
parser.add_argument("--room", type=float, default=4.0)
parser.add_argument("--seconds", type=float, default=0.0, help="0 = 창을 닫을 때까지")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb  # noqa: E402
import omni  # noqa: E402
import torch  # noqa: E402

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from scene import build_world, set_camera, settle, spawn_robot  # noqa: E402
from tasks.common import robot_spec as spec  # noqa: E402
from tasks.rl_correct.rl_correct_env import wheel_targets  # noqa: E402

DT = 1 / 120.0          # 물리 1스텝 = 8.33 ms
CMD_DT = 1 / 20.0       # 명령은 20 Hz (cmd_manager와 같게)
SPEED = 0.12            # [m/s] 일정


class Keyboard:
    DOWN = 1 << 3
    MAP = {f"{d}": getattr(carb.input.KeyboardInput, f"KEY_{d}") for d in "01248"}

    def __init__(self):
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()

    @property
    def pressed(self) -> set[str]:
        flags = self._input.get_keyboard_button_flags
        return {name for name, key in self.MAP.items() if flags(self._keyboard, key) & self.DOWN}


def main():
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))
    set_camera(sim)
    build_world(1 if args_cli.room > 0 else 0, args_cli.room, 0)
    scene, robot, _ = spawn_robot()
    sim.reset()
    settle(scene, robot, sim)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    dev = robot.device
    r = torch.tensor([spec.WHEEL_RADIUS], device=dev)
    sep = torch.tensor([spec.WHEEL_SEP], device=dev)
    print("[l2] 같은 명령 0.12 m/s · 물리 dt=1/120 s · 명령 20 Hz")
    print("[l2] 1/2/4/8 = 한 프레임당 물리 스텝 수 · 0 = 정지")

    kb = Keyboard()
    steps_per_frame = 2
    moving = True
    sim_t = 0.0
    next_cmd = 0.0
    prev_keys = set()
    t0 = time.perf_counter()
    last_print = 0.0
    while True:
        frame_start = time.perf_counter()
        wall = frame_start - t0
        if args_cli.seconds > 0 and wall >= args_cli.seconds:
            break
        if args_cli.seconds == 0 and not simulation_app.is_running():
            break

        keys = kb.pressed
        new = keys - prev_keys
        prev_keys = keys
        if "0" in new:
            moving = False
            print("[l2] 정지")
        for k in ("1", "2", "4", "8"):
            if k in new:
                steps_per_frame = int(k)
                moving = True
                print(f"[l2] steps/frame = {steps_per_frame}")

        if sim_t >= next_cmd:
            next_cmd += CMD_DT
            v = SPEED if moving else 0.0
            wheels = wheel_targets(torch.tensor([v], device=dev), torch.tensor([0.0], device=dev),
                                   r, sep, spec.MAX_WHEEL)
            robot.set_joint_velocity_target(wheels, joint_ids=wheel_ids)
            scene.write_data_to_sim()

        for _ in range(steps_per_frame):
            sim.step(render=False)
        scene.update(steps_per_frame * DT)
        sim.render()
        sim_t += steps_per_frame * DT

        if wall - last_print >= 1.0:
            last_print = wall
            x = robot.data.root_pos_w.torch[0, 0].item()
            ratio = sim_t / max(wall, 1e-6)
            print(f"[l2] steps/frame={steps_per_frame} · sim {sim_t:.2f}s / wall {wall:.2f}s "
                  f"({ratio:.2f}x) · x={x:+.2f} m")

        rest = 1 / 60 - (time.perf_counter() - frame_start)
        if rest > 0:
            time.sleep(rest)

    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
