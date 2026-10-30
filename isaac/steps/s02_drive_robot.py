"""S2 — 키보드로 로봇을 직접 몰아 본다 (실시간, 20 Hz 명령 = cmd_manager와 같게).

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./steps/run.sh s02_drive_robot.py

키:
  W / ↑  전진        S / ↓  후진
  A / ←  좌회전(+)   D / →  우회전(−)
  Shift  느리게(40%)   X  정지   R  처음 위치로
  N  다음 랜덤 방 (방마다 장애물 배치가 다름, --seed로 재현)
  (창을 닫으면 종료)

바퀴 명령은 실제 드라이버와 같은 식(rl.task.rl_correct_env.wheel_targets:
반지름 0.033 · 간격 0.287 · 8 rad/s 포화)을 쓴다. 물리 120 Hz, 명령 20 Hz, 화면 60 Hz.
"""
import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="S2: 키보드로 우리 로봇 몰기")
parser.add_argument("--room", type=float, default=4.0, help="방 한 변 길이 [m] (0 = 벽 없이)")
parser.add_argument("--rooms", type=int, default=6, help="랜덤 방 개수 (N 키로 이동)")
parser.add_argument("--seed", type=int, default=0, help="장애물 배치 시드 (같으면 재현)")
parser.add_argument("--speed", type=float, default=0.15, help="전진 속도 [m/s] (최대 0.26)")
parser.add_argument("--turn", type=float, default=1.0, help="회전 속도 [rad/s] (최대 1.82)")
parser.add_argument("--seconds", type=float, default=0.0, help="0 = 창을 닫을 때까지")
parser.add_argument("--demo", action="store_true", help="키보드 대신 정해진 패턴(앞 2초 → 좌회전 2초 → 정지)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb  # noqa: E402
import omni  # noqa: E402
import torch  # noqa: E402

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from scene import (FRICTION_GROUND, FRICTION_OBSTACLE, FRICTION_WHEEL,
                   build_world, place_robot, set_camera, settle, spawn_robot)  # noqa: E402
from tasks.common import robot_spec as spec  # noqa: E402
from rl.task.rl_correct_env import wheel_targets  # noqa: E402

RENDER_DT = 1.0 / 60.0
POLICY_DT = 1.0 / spec.CONTROL_HZ     # 20 Hz — 실제 cmd_manager와 같게
FORWARD_KEYS = ("W", "UP")
BACK_KEYS = ("S", "DOWN")
LEFT_KEYS = ("A", "LEFT")
RIGHT_KEYS = ("D", "RIGHT")
SLOW_KEYS = ("LEFT_SHIFT", "RIGHT_SHIFT")
STOP_KEYS = ("X",)


class Keyboard:
    """지금 눌린 키 집합 — 이벤트 구독 대신 상태를 직접 조회한다.

    Kit 버전에 따라 KeyboardEvent.input이 enum이 아니라 str로 오는 경우가 있어
    (2026-10-30 실측: `'str' object has no attribute 'name'`), 이벤트를 해석하지 않고
    `get_keyboard_button_flags`로 현재 눌림 상태(bit 3 = down)를 읽는다.
    키를 놓친 채 포커스가 빠져도 stuck 키가 없다.
    """

    DOWN = 1 << 3  # carb/input/InputTypes.h: kButtonFlagStateDown
    MAP = {
        "W": carb.input.KeyboardInput.W, "A": carb.input.KeyboardInput.A,
        "S": carb.input.KeyboardInput.S, "D": carb.input.KeyboardInput.D,
        "UP": carb.input.KeyboardInput.UP, "DOWN": carb.input.KeyboardInput.DOWN,
        "LEFT": carb.input.KeyboardInput.LEFT, "RIGHT": carb.input.KeyboardInput.RIGHT,
        "LEFT_SHIFT": carb.input.KeyboardInput.LEFT_SHIFT,
        "RIGHT_SHIFT": carb.input.KeyboardInput.RIGHT_SHIFT,
        "X": carb.input.KeyboardInput.X, "R": carb.input.KeyboardInput.R,
        "N": carb.input.KeyboardInput.N,
    }

    def __init__(self):
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()

    @property
    def pressed(self) -> set[str]:
        flags = self._input.get_keyboard_button_flags
        return {name for name, key in self.MAP.items() if flags(self._keyboard, key) & self.DOWN}

    def close(self):
        pass


def held(pressed, keys) -> bool:
    return any(k in pressed for k in keys)


def main():
    sim = SimulationContext(SimulationCfg(dt=1 / 120, device=args_cli.device))
    set_camera(sim)
    rooms = build_world(args_cli.rooms if args_cli.room > 0 else 0, args_cli.room, args_cli.seed)
    scene, robot, _ = spawn_robot()
    sim.reset()
    z = settle(scene, robot, sim)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    device = robot.device
    r = torch.tensor([spec.WHEEL_RADIUS], device=device)
    sep = torch.tensor([spec.WHEEL_SEP], device=device)
    print(f"[s2] base 높이: {z:.3f} m")
    print("[s2] W/↑ 전진 · S/↓ 후진 · A/← 좌회전 · D/→ 우회전 · Shift 느리게 · X 정지 · R 리셋 · N 다음 방")
    if rooms:
        r0 = rooms[0]
        print(f"[s2] 방 1/{len(rooms)} · seed {r0.seed} · 장애물 {r0.obstacles}개")
    print(f"[s2] 마찰: 바닥 {FRICTION_GROUND} · 장애물 {FRICTION_OBSTACLE} · 바퀴 {FRICTION_WHEEL} (static, dynamic)")

    kb = None if args_cli.demo else Keyboard()
    reset_latch = False
    next_latch = False
    room_idx = 0
    t0 = time.perf_counter()
    sim_t = 0.0                 # 시뮬 시간
    sim_dt = 1 / 120
    next_policy = 0.0           # 벽시계 기준 20 Hz
    frames = 0
    cmd = (0.0, 0.0)
    printed = None

    while simulation_app.is_running():
        frame_start = time.perf_counter()
        wall = frame_start - t0
        if args_cli.seconds > 0 and wall > args_cli.seconds:
            break

        # ---- 명령 (20 Hz, 벽시계) — 바퀴 목표를 바꾸고 한 번만 sim에 씀
        if wall >= next_policy:
            next_policy += POLICY_DT
            if args_cli.demo:
                if wall < 2.0:
                    v, w = args_cli.speed, 0.0
                elif wall < 4.0:
                    v, w = 0.0, args_cli.turn
                else:
                    v, w = 0.0, 0.0
            else:
                p = kb.pressed
                v = (held(p, FORWARD_KEYS) - held(p, BACK_KEYS)) * args_cli.speed
                w = (held(p, LEFT_KEYS) - held(p, RIGHT_KEYS)) * args_cli.turn
                if held(p, SLOW_KEYS):
                    v, w = v * 0.4, w * 0.4
                if held(p, STOP_KEYS):
                    v, w = 0.0, 0.0
                if "R" in p and not reset_latch:      # 리셋은 누를 때 한 번만
                    origin = rooms[room_idx].origin if rooms else (0.0, 0.0, 0.0)
                    place_robot(robot, origin, 0.0, wheel_ids)
                    v, w = 0.0, 0.0
                reset_latch = "R" in p
                if "N" in p and not next_latch and rooms:   # 다음 랜덤 방
                    room_idx = (room_idx + 1) % len(rooms)
                    room = rooms[room_idx]
                    place_robot(robot, room.origin, 0.0, wheel_ids)
                    set_camera(sim, room.origin)
                    print(f"[s2] 방 {room_idx + 1}/{len(rooms)} · seed {room.seed} · 장애물 {room.obstacles}개")
                    v, w = 0.0, 0.0
                next_latch = "N" in p
            v = max(-spec.MAX_LIN, min(spec.MAX_LIN, v))
            w = max(-spec.MAX_ANG, min(spec.MAX_ANG, w))
            wheels = wheel_targets(torch.tensor([v], device=device), torch.tensor([w], device=device),
                                   r, sep, spec.MAX_WHEEL)
            robot.set_joint_velocity_target(wheels, joint_ids=wheel_ids)
            scene.write_data_to_sim()
            scene.update(POLICY_DT)
            cmd = (v, w)

            shown = (round(v, 2), round(w, 2))
            if shown != printed:
                print(f"[s2] 명령 v={v:+.2f} m/s  w={w:+.2f} rad/s")
                printed = shown

        # ---- 물리: 벽시계를 따라잡을 만큼 120 Hz로 (렌더는 프레임당 1번)
        #      렌더가 느려도 물리 속도는 실시간 유지. 한 프레임 최대 16스텝(133 ms)까지.
        target = min(wall, sim_t + 16 * sim_dt)
        steps = 0
        while sim_t + sim_dt <= target:
            sim.step(render=False)
            sim_t += sim_dt
            steps += 1
        sim.render()
        frames += 1

        rest = RENDER_DT - (time.perf_counter() - frame_start)
        if rest > 0:
            time.sleep(rest)

    scene.update(1 / 120)
    x, y = robot.data.root_pos_w.torch[0, :2].tolist()
    yaw = robot.data.heading_w.torch[0].item()
    elapsed = time.perf_counter() - t0
    print(f"[s2] 끝 위치 x={x:+.2f} m y={y:+.2f} m yaw={yaw:+.2f} rad")
    print(f"[s2] sim {sim_t:.1f} s / {frames} frames / 벽시계 {elapsed:.1f} s ({sim_t / max(elapsed, 1e-6):.2f}×)")
    if kb is not None:
        kb.close()
    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
