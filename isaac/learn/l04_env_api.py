"""L4 — Env API: obs · action · reward · done · reset.

RL 환경의 한 스텝(20 Hz)이 이렇게 흐른다:

  obs = [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]     (6개)
  action = [dV, dW]  ([-1,1], 실제 보정량 = action × (0.10 m/s, 0.60 rad/s))
  corrected = cmd + action → 바퀴 rad/s → 물리 6스텝(120 Hz)
  reward = -( |v_cmd−v_true| + 0.5|w_cmd−w_true| ) − 0.01|a|² − 0.05|a−a_prev|²
  done = 10초 → reset

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./learn/run.sh l04_env_api.py

키:
  1  action [0, 0]      (보정 없음)
  2  action [+0.2, 0]   (앞으로 더, +0.02 m/s)
  3  action [-0.2, 0]   (앞으로 덜)
  4  action [0, +0.2]   (왼쪽으로 더, +0.12 rad/s)
  5  action [0, -0.2]   (오른쪽으로 더)
  0  action [0, 0]
  R  reset

콘솔: 0.5초마다 obs/action/reward/done 출력.
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

parser = argparse.ArgumentParser(description="L4: Env API")
parser.add_argument("--room", type=float, default=0.0, help="0 = 벽 없이")
parser.add_argument("--steps", type=int, default=0, help="0 = 창을 닫을 때까지, >0 = 그 스텝만")
parser.add_argument("--action", type=float, nargs=2, default=[0.0, 0.0], help="시작 action (헤드리스 테스트용)")
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

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from scene import build_world, place_robot, set_camera, settle, spawn_robot  # noqa: E402
from tasks.common import robot_spec as spec  # noqa: E402
from rl.task.rl_correct_env import wheel_targets  # noqa: E402

DT = 1 / 120.0
POLICY_DT = 1 / spec.CONTROL_HZ      # 0.05 s
SUBSTEPS = round(POLICY_DT / DT)     # 6
EPISODE_S = 10.0
ACTION_SCALE = (0.10, 0.60)          # [m/s, rad/s] — rl_correct v2와 같음

# (시작 시각, v_cmd, w_cmd) — 2초마다 바뀜
CMDS = (
    (0.0, 0.12, 0.0),    # 직진
    (2.0, 0.0, 0.8),     # 제자리 좌회전
    (4.0, 0.10, 0.4),    # 좌회전 원호
    (6.0, 0.0, -0.8),    # 제자리 우회전
    (8.0, 0.0, 0.0),     # 정지
)


class Keyboard:
    DOWN = 1 << 3

    def __init__(self):
        self.MAP = {d: getattr(carb.input.KeyboardInput, f"KEY_{d}") for d in "012345"}
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()

    @property
    def pressed(self) -> set[str]:
        flags = self._input.get_keyboard_button_flags
        return {name for name, key in self.MAP.items() if flags(self._keyboard, key) & self.DOWN}


class ToyEnv:
    """우리 로봇 + rl_correct와 같은 obs/action/reward를 가진 최소 환경."""

    def __init__(self, scene, robot, sim, wheel_ids):
        self.scene, self.robot, self.sim = scene, robot, sim
        self.wheel_ids = wheel_ids
        self.dev = robot.device
        self.r = torch.tensor([spec.WHEEL_RADIUS], device=self.dev)
        self.sep = torch.tensor([spec.WHEEL_SEP], device=self.dev)
        self.t = 0.0
        self.t_change = 0.0
        self.next_change = 2.0
        self.cmd = (0.0, 0.0)
        self.prev_action = torch.zeros(2, device=self.dev)

    def _cmd_at(self, t: float):
        cmd = CMDS[0]
        for c in CMDS:
            if t >= c[0]:
                cmd = c
        return cmd[1], cmd[2]

    def _obs(self):
        jv = self.robot.data.joint_vel.torch[0, self.wheel_ids].tolist()
        v_enc = spec.WHEEL_RADIUS * (jv[0] + jv[1]) / 2.0
        w_enc = spec.WHEEL_RADIUS * (jv[1] - jv[0]) / spec.WHEEL_SEP
        w_gyro = self.robot.data.root_ang_vel_b.torch[0, 2].item()
        return [self.cmd[0], self.cmd[1], v_enc, w_enc, w_gyro, min(self.t_change, 2.0)]

    def reset(self):
        place_robot(self.robot, (0.0, 0.0, 0.0), 0.0, self.wheel_ids)
        self.scene.update(DT)
        self.t = 0.0
        self.t_change = 0.0
        self.next_change = 2.0
        self.cmd = self._cmd_at(0.0)
        self.prev_action = torch.zeros(2, device=self.dev)
        return self._obs()

    def step(self, action: torch.Tensor):
        action = action.clamp(-1.0, 1.0)
        moving = abs(self.cmd[0]) + abs(self.cmd[1]) > 1e-6
        d = action * torch.tensor(ACTION_SCALE, device=self.dev) if moving else torch.zeros(2, device=self.dev)
        v = max(-spec.MAX_LIN, min(spec.MAX_LIN, self.cmd[0] + d[0].item()))
        w = max(-spec.MAX_ANG, min(spec.MAX_ANG, self.cmd[1] + d[1].item()))
        wheels = wheel_targets(torch.tensor([v], device=self.dev), torch.tensor([w], device=self.dev),
                               self.r, self.sep, spec.MAX_WHEEL)
        self.robot.set_joint_velocity_target(wheels, joint_ids=self.wheel_ids)
        self.scene.write_data_to_sim()
        for _ in range(SUBSTEPS):
            self.sim.step(render=False)
        self.scene.update(POLICY_DT)
        self.sim.render()

        self.t += POLICY_DT
        self.t_change += POLICY_DT
        if self.t >= self.next_change and self.next_change < EPISODE_S:
            self.next_change += 2.0
            self.cmd = self._cmd_at(self.t)
            self.t_change = 0.0

        obs = self._obs()
        v_true = self.robot.data.root_lin_vel_b.torch[0, 0].item()
        w_true = self.robot.data.root_ang_vel_b.torch[0, 2].item()
        err = abs(self.cmd[0] - v_true) + 0.5 * abs(self.cmd[1] - w_true)
        penalty = 0.01 * (action ** 2).sum().item() + 0.05 * ((action - self.prev_action) ** 2).sum().item()
        reward = -err - penalty
        self.prev_action = action.clone()
        return obs, reward, self.t >= EPISODE_S


def main():
    headless = not getattr(args_cli, "visualizer", None) or "kit" not in (args_cli.visualizer or [])
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))
    set_camera(sim)
    build_world(1 if args_cli.room > 0 else 0, args_cli.room, 0)
    scene, robot, _ = spawn_robot()
    sim.reset()
    settle(scene, robot, sim)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)

    env = ToyEnv(scene, robot, sim, wheel_ids)
    kb = Keyboard() if (HAS_INPUT and not headless) else None
    obs = env.reset()
    action = torch.tensor(args_cli.action, device=robot.device)
    step = 0
    prev_keys = set()
    t0 = time.perf_counter()
    print("[l4] obs = [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]")
    print("[l4] 1=보정없음 · 2=앞+ · 3=앞- · 4=좌+ · 5=우- · 0=0 · R=reset")

    while True:
        frame_start = time.perf_counter()
        wall = frame_start - t0
        if args_cli.steps > 0 and step >= args_cli.steps:
            break
        if args_cli.steps == 0 and not simulation_app.is_running():
            break

        if kb is not None:
            keys = kb.pressed
            new = keys - prev_keys
            prev_keys = keys
            if "1" in new or "0" in new:
                action = torch.tensor(args_cli.action, device=robot.device)
                print("[l4] action = [0, 0]")
            elif "2" in new:
                action = torch.tensor([0.2, 0.0], device=robot.device)
                print("[l4] action = [+0.2, 0]")
            elif "3" in new:
                action = torch.tensor([-0.2, 0.0], device=robot.device)
                print("[l4] action = [-0.2, 0]")
            elif "4" in new:
                action = torch.tensor([0.0, 0.2], device=robot.device)
                print("[l4] action = [0, +0.2]")
            elif "5" in new:
                action = torch.tensor([0.0, -0.2], device=robot.device)
                print("[l4] action = [0, -0.2]")
            if "R" in new:
                obs = env.reset()
                step = 0
                print("[l4] reset")

        obs, reward, done = env.step(action)
        step += 1
        if step % 10 == 0:
            o = " ".join(f"{x:+.2f}" for x in obs)
            print(f"[l4] step {step:3d} t={env.t:4.1f}s")
            print(f"     obs  [{o}]")
            print(f"     act  [{action[0]:+.2f},{action[1]:+.2f}]  rew {reward:+.3f}  done {done}")
        if done:
            print("[l4] episode done → reset")
            obs = env.reset()
            step = 0

        rest = POLICY_DT - (time.perf_counter() - frame_start)
        if rest > 0:
            time.sleep(rest)

    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
