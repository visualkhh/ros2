"""01 — 랜덤 환경에서 우리 로봇 띄우기.

방 크기(3.5~6.0 m)와 장애물(3~6개) 배치가 방마다 랜덤.
방 8개를 미리 만들어 두고 N으로 로봇만 순간이동한다 (프림 삭제/재생성 없음).

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./steps/01_random_world.sh

키:
  W/↑ 전진 · S/↓ 후진 · A/← 좌회전 · D/→ 우회전
  Shift 느리게 · X 정지 · R 현재 방 리셋 · N 다음 랜덤 방
"""
import argparse
import random
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ISAAC))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="S6: 랜덤 환경 + 우리 로봇")
parser.add_argument("--rooms", type=int, default=8, help="미리 만들 랜덤 방 수")
parser.add_argument("--seed", type=int, default=None, help="랜덤 시드 (없으면 매번 새로)")
parser.add_argument("--size-min", type=float, default=3.5, help="방 최소 한 변 [m]")
parser.add_argument("--size-max", type=float, default=6.0, help="방 최대 한 변 [m]")
parser.add_argument("--speed", type=float, default=0.15, help="전진 속도 [m/s]")
parser.add_argument("--turn", type=float, default=1.0, help="회전 속도 [rad/s]")
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

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from scene import build_world, place_robot, set_camera, settle, spawn_robot  # noqa: E402
from tasks.common import robot_spec as spec  # noqa: E402
from tasks.common.sim_params import WheelLag, describe as calib_desc  # noqa: E402
from rl.wheel_drift.task.rl_correct_env import wheel_targets  # noqa: E402

DT = 1 / 120.0
POLICY_DT = 1 / spec.CONTROL_HZ
FRAME_DT = 1 / 60.0

FORWARD_KEYS = ("W", "UP")
BACK_KEYS = ("S", "DOWN")
LEFT_KEYS = ("A", "LEFT")
RIGHT_KEYS = ("D", "RIGHT")
SLOW_KEYS = ("LEFT_SHIFT", "RIGHT_SHIFT")


class Keyboard:
    DOWN = 1 << 3

    def __init__(self):
        self.MAP = {
            "W": carb.input.KeyboardInput.W, "A": carb.input.KeyboardInput.A,
            "S": carb.input.KeyboardInput.S, "D": carb.input.KeyboardInput.D,
            "UP": carb.input.KeyboardInput.UP, "DOWN": carb.input.KeyboardInput.DOWN,
            "LEFT": carb.input.KeyboardInput.LEFT, "RIGHT": carb.input.KeyboardInput.RIGHT,
            "LEFT_SHIFT": carb.input.KeyboardInput.LEFT_SHIFT,
            "RIGHT_SHIFT": carb.input.KeyboardInput.RIGHT_SHIFT,
            "X": carb.input.KeyboardInput.X, "R": carb.input.KeyboardInput.R,
            "N": carb.input.KeyboardInput.N,
        }
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()

    @property
    def pressed(self) -> set[str]:
        flags = self._input.get_keyboard_button_flags
        return {name for name, key in self.MAP.items() if flags(self._keyboard, key) & self.DOWN}


def held(pressed, keys) -> bool:
    return any(k in pressed for k in keys)


def main():
    headless = not getattr(args_cli, "visualizer", None) or "kit" not in (args_cli.visualizer or [])
    seed = args_cli.seed if args_cli.seed is not None else random.randrange(1_000_000)
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))
    rooms = build_world(args_cli.rooms, args_cli.size_min, seed,
                        size_range=(args_cli.size_min, args_cli.size_max))
    set_camera(sim, rooms[0].origin if rooms else (0, 0, 0))
    scene, robot, _ = spawn_robot()
    sim.reset()
    settle(scene, robot, sim)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    dev = robot.device
    r = torch.tensor([spec.WHEEL_RADIUS], device=dev)
    sep = torch.tensor([spec.WHEEL_SEP], device=dev)
    wheel_model = WheelLag(dt=POLICY_DT)

    room_idx = 0
    place_robot(robot, rooms[room_idx].origin if rooms else (0, 0, 0), 0.0, wheel_ids)
    set_camera(sim, rooms[room_idx].origin if rooms else (0, 0, 0))

    def show_room():
        if rooms:
            rm = rooms[room_idx]
            print(f"[01] seed {seed} · 방 {room_idx + 1}/{len(rooms)} · "
                  f"{rm.size:.1f} m · 장애물 {rm.obstacles}개")

    show_room()
    print(f"[01] calib: {calib_desc()}")
    print("[01] W/↑ 전진 · S/↓ 후진 · A/← 좌회전 · D/→ 우회전 · Shift 느리게 · X 정지 · R 리셋 · N 다음 방")

    kb = None if headless else Keyboard()
    target = (0.0, 0.0)
    prev_keys = set()
    reset_latch = next_latch = False
    next_cmd = 0.0
    sim_t = 0.0
    t0 = time.perf_counter()
    while True:
        frame_start = time.perf_counter()
        wall = frame_start - t0
        if args_cli.seconds > 0 and wall >= args_cli.seconds:
            break
        if args_cli.seconds == 0 and not simulation_app.is_running():
            break

        if sim_t >= next_cmd:
            next_cmd += POLICY_DT
            if kb is not None:
                p = kb.pressed
                v = (held(p, FORWARD_KEYS) - held(p, BACK_KEYS)) * args_cli.speed
                w = (held(p, LEFT_KEYS) - held(p, RIGHT_KEYS)) * args_cli.turn
                if held(p, SLOW_KEYS):
                    v, w = v * 0.4, w * 0.4
                if "X" in p:
                    v, w = 0.0, 0.0
                if "R" in p and not reset_latch:
                    place_robot(robot, rooms[room_idx].origin if rooms else (0, 0, 0), 0.0, wheel_ids)
                    wheel_model.reset()
                    v, w = 0.0, 0.0
                reset_latch = "R" in p
                if "N" in p and not next_latch and rooms:
                    room_idx = (room_idx + 1) % len(rooms)
                    place_robot(robot, rooms[room_idx].origin, 0.0, wheel_ids)
                    set_camera(sim, rooms[room_idx].origin)
                    wheel_model.reset()
                    show_room()
                    v, w = 0.0, 0.0
                next_latch = "N" in p
                target = (max(-spec.MAX_LIN, min(spec.MAX_LIN, v)),
                          max(-spec.MAX_ANG, min(spec.MAX_ANG, w)))
            wheels = wheel_model.step(target[0], target[1], wheel_targets, r, sep,
                                      spec.MAX_WHEEL, dev)
            robot.set_joint_velocity_target(wheels, joint_ids=wheel_ids)
            scene.write_data_to_sim()
            scene.update(POLICY_DT)

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
