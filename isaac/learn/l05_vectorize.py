"""L5 — Vectorization: 로봇 여러 대를 한 번에.

Isaac Lab은 `{ENV_REGEX_NS}/Robot`으로 로봇을 만들면
`/World/envs/env_0/Robot`, `env_1/Robot`, ... 로 자동 복제한다.
`sim.step()` 한 번이 **모든 env**를 동시에 진행한다.

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./learn/run.sh l05_vectorize.py --envs 16

키:
  1  전진 (v=0.12, w=0)
  2  제자리 좌회전 (v=0, w=+0.6)
  3  제자리 우회전 (v=0, w=-0.6)
  0  정지
  R  전체 reset

콘솔: 1초마다 env 수, env_origins, env0/envN 위치·yaw, sim/wall 비율.
"""
import argparse
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ISAAC))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="L5: 병렬 환경")
parser.add_argument("--envs", type=int, default=16, help="로봇 수")
parser.add_argument("--spacing", type=float, default=1.5, help="env 간격 [m]")
parser.add_argument("--seconds", type=float, default=0.0, help="0 = 창을 닫을 때까지")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb  # noqa: E402

try:
    import carb.input  # noqa: F401
    HAS_INPUT = True
except Exception:
    HAS_INPUT = False

import omni  # noqa: E402
import torch  # noqa: E402

from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from scene import bind_robot_materials, build_world, robot_articulation_cfg, settle  # noqa: E402
from tasks.common import robot_spec as spec  # noqa: E402
from rl.wheel_drift.task.rl_correct_env import wheel_targets  # noqa: E402

DT = 1 / 120.0
FRAME_DT = 1 / 60.0


class Keyboard:
    DOWN = 1 << 3

    def __init__(self):
        self.MAP = {d: getattr(carb.input.KeyboardInput, f"KEY_{d}") for d in "0123"}
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()

    @property
    def pressed(self) -> set[str]:
        flags = self._input.get_keyboard_button_flags
        return {name for name, key in self.MAP.items() if flags(self._keyboard, key) & self.DOWN}


def main():
    headless = not getattr(args_cli, "visualizer", None) or "kit" not in (args_cli.visualizer or [])
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))

    side = math.ceil(math.sqrt(args_cli.envs))
    half = side * args_cli.spacing / 2.0
    sim.set_camera_view(eye=(half * 1.9, -half * 1.9, half * 1.6), target=(0.0, 0.0, 0.0))

    build_world(0, 0.0, 0)
    robot_cfg, usd_path = robot_articulation_cfg(prim_path="{ENV_REGEX_NS}/Robot")
    print(f"[l5] robot USD -> {usd_path}", flush=True)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        num_envs = args_cli.envs
        env_spacing = args_cli.spacing
        robot = robot_cfg

    scene = InteractiveScene(SceneCfg())
    robot = scene["robot"]
    bind_robot_materials(scene, prim_path="/World/envs")
    sim.reset()
    scene.reset()
    settle(scene, robot, sim)

    n = scene.num_envs
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    dev = robot.device
    r = torch.tensor([spec.WHEEL_RADIUS], device=dev)
    sep = torch.tensor([spec.WHEEL_SEP], device=dev)
    v = torch.zeros(n, device=dev)
    w = torch.zeros(n, device=dev)
    print(f"[l5] envs={n} · spacing={args_cli.spacing} m · scene.env_origins={tuple(scene.env_origins[0].tolist())}")
    print("[l5] 1=전진 · 2=좌회전 · 3=우회전 · 0=정지 · R=reset")

    kb = Keyboard() if (HAS_INPUT and not headless) else None
    prev_keys = set()
    sim_t = 0.0
    last_print = 0.0
    frames = 0
    t0 = time.perf_counter()
    while True:
        frame_start = time.perf_counter()
        wall = frame_start - t0
        if args_cli.seconds > 0 and wall >= args_cli.seconds:
            break
        if args_cli.seconds == 0 and not simulation_app.is_running():
            break

        if kb is not None:
            keys = kb.pressed
            new = keys - prev_keys
            prev_keys = keys
            if "1" in new:
                v[:] = 0.12; w[:] = 0.0
                print("[l5] 전진")
            elif "2" in new:
                v[:] = 0.0; w[:] = 0.6
                print("[l5] 좌회전")
            elif "3" in new:
                v[:] = 0.0; w[:] = -0.6
                print("[l5] 우회전")
            elif "0" in new:
                v[:] = 0.0; w[:] = 0.0
                print("[l5] 정지")
            if "R" in new:
                scene.reset()
                sim_t = 0.0
                v[:] = 0.0; w[:] = 0.0
                print("[l5] reset")

        wheels = wheel_targets(v, w, r, sep, spec.MAX_WHEEL)
        robot.set_joint_velocity_target(wheels, joint_ids=wheel_ids)
        scene.write_data_to_sim()
        sim.step(render=False)
        sim.step(render=False)
        scene.update(2 * DT)
        sim.render()
        sim_t += 2 * DT
        frames += 1

        if wall - last_print >= 1.0:
            last_print = wall
            pos = robot.data.root_pos_w.torch
            yaw = robot.data.heading_w.torch
            ratio = sim_t / max(wall, 1e-6)
            fps = frames / max(wall, 1e-6)
            env_steps_s = n * ratio / DT
            print(f"[l5] sim {sim_t:4.1f}s / wall {wall:4.1f}s ({ratio:.2f}x) · "
                  f"fps {fps:.0f} · env-steps/s {env_steps_s:.0f} · "
                  f"env0=({pos[0,0]:+.2f},{pos[0,1]:+.2f},{yaw[0]:+.2f}) · "
                  f"env{n-1}=({pos[-1,0]:+.2f},{pos[-1,1]:+.2f},{yaw[-1]:+.2f})")

        rest = FRAME_DT - (time.perf_counter() - frame_start)
        if rest > 0:
            time.sleep(rest)

    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
