"""01 — 랜덤 환경에서 우리 로봇 띄우기.

방 크기(3.5~6.0 m)와 장애물(3~6개) 배치가 방마다 랜덤.
방 8개를 미리 만들어 두고 N으로 로봇만 순간이동한다 (프림 삭제/재생성 없음).

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./steps/01_random_world.sh

키:
  W/↑ 전진 · S/↓ 후진 · A/← 좌회전 · D/→ 우회전
  Shift 느리게 · X 정지 · R 현재 방 리셋 · N 다음 랜덤 방 · C 보정값 창
  보정값 창: 숫자 수정 즉시 적용 · 0 set(전부 0) · reset(시작값 복구)
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
from tasks.common.sim_params import WheelLag, describe as calib_desc, values as calib_values  # noqa: E402
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
            "N": carb.input.KeyboardInput.N, "C": carb.input.KeyboardInput.C,
        }
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()

    @property
    def pressed(self) -> set[str]:
        flags = self._input.get_keyboard_button_flags
        return {name for name, key in self.MAP.items() if flags(self._keyboard, key) & self.DOWN}


def held(pressed, keys) -> bool:
    return any(k in pressed for k in keys)


class CalibHUD:
    """실행 중 sim 보정값을 보고 고치는 창 (GUI 전용).

    - 숫자 필드는 수정 즉시 sim에 반영된다 (JSON에는 저장하지 않음).
    - `0 set` → 표시된 숫자 전부 0.
    - `reset` → 이 창을 열 때 읽은 값(JSON 또는 DEFAULTS)으로 복구.
    """

    LABEL_STYLE = {"font_size": 13, "color": 0xFF9FB4C7}
    VALUE_STYLE = {"font_size": 13, "color": 0xFFF0F0F0}
    HEAD_STYLE = {"font_size": 12, "color": 0xFFF0C040}

    def __init__(self, condition, wheel_model, robot, wheel_ids, stage, device):
        import omni.ui as ui

        self.ui = ui
        self.wheel_model = wheel_model
        self.robot = robot
        self.wheel_ids = wheel_ids
        self.stage = stage
        self.device = device
        self._syncing = False
        self.models = {}
        self.kinds = {}

        d = calib_values(condition)
        self.start = {
            "tau": float(wheel_model.tau),
            "delay_steps": int(wheel_model.delay_steps),
            "gain": float(wheel_model.gain),
            "deadband": float(wheel_model.deadband),
            "damping": float(d["actuator_damping"]),
            "effort": float(d["actuator_effort"]),
            "ground_s": float(d["friction_ground"][0]),
            "ground_d": float(d["friction_ground"][1]),
            "wheel_s": float(d["friction_wheel"][0]),
            "wheel_d": float(d["friction_wheel"][1]),
            "caster_s": float(d["friction_caster"][0]),
            "caster_d": float(d["friction_caster"][1]),
            "obstacle_s": float(d["friction_obstacle"][0]),
            "obstacle_d": float(d["friction_obstacle"][1]),
        }
        self.fric_paths = {
            "ground": "/World/ground/physicsMaterial",
            "wheel": "/World/Materials/wheel",
            "caster": "/World/Materials/caster",
            "obstacle": "/World/Materials/obstacle",
        }

        self.window = ui.Window("SIM CALIBRATION", width=450, height=540)
        try:
            self.window.position_x, self.window.position_y = 18, 48
        except Exception:
            pass
        with self.window.frame:
            with ui.VStack(spacing=4):
                self._heading("WHEEL MODEL")
                self._row("tau [s]", "tau", "float")
                self._row("dead steps", "delay_steps", "int")
                self._row("gain", "gain", "float")
                self._row("deadband [rad/s]", "deadband", "float")

                self._heading("ACTUATOR")
                self._row("damping", "damping", "float")
                self._row("effort [N.m]", "effort", "float")

                self._heading("FRICTION (static / dynamic)")
                self._pair("ground", "ground_s", "ground_d")
                self._pair("wheel", "wheel_s", "wheel_d")
                self._pair("caster", "caster_s", "caster_d")
                self._pair("obstacle", "obstacle_s", "obstacle_d")

                with ui.HStack(spacing=6, height=30):
                    ui.Button("0 set", clicked_fn=self.zero, width=110)
                    ui.Button("reset", clicked_fn=self.reset, width=110)

                self.status = ui.Label(f"source: {d.get('source', 'defaults')}",
                                       height=20, style=self.LABEL_STYLE)

    def _heading(self, text):
        self.ui.Label(text, height=20, style=self.HEAD_STYLE)

    def _row(self, label, key, kind):
        ui = self.ui
        with ui.HStack(height=22):
            ui.Label(label, width=145, style=self.LABEL_STYLE)
            if kind == "int":
                model = ui.SimpleIntModel(int(self.start[key]))
                ui.IntField(model, width=120)
            else:
                model = ui.SimpleFloatModel(float(self.start[key]))
                ui.FloatField(model, width=120)
            model.add_value_changed_fn(lambda _m: self.apply())
        self.models[key] = model
        self.kinds[key] = kind

    def _pair(self, label, key_s, key_d):
        ui = self.ui
        with ui.HStack(height=22):
            ui.Label(label, width=145, style=self.LABEL_STYLE)
            for key in (key_s, key_d):
                model = ui.SimpleFloatModel(float(self.start[key]))
                ui.FloatField(model, width=65)
                model.add_value_changed_fn(lambda _m: self.apply())
                self.models[key] = model
                self.kinds[key] = "float"
            ui.Label("static / dynamic", width=120, style=self.LABEL_STYLE)

    def _num(self, key):
        if self.kinds[key] == "int":
            return float(self.models[key].get_value_as_int())
        return float(self.models[key].get_value_as_float())

    def _set_all(self, values):
        self._syncing = True
        try:
            for key, model in self.models.items():
                val = float(values.get(key, 0.0))
                model.set_value(int(round(val)) if self.kinds[key] == "int" else val)
        finally:
            self._syncing = False

    def _set_friction(self, path, static, dynamic):
        from pxr import Sdf, UsdPhysics

        prim = self.stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return False
        if not prim.HasAPI(UsdPhysics.MaterialAPI):
            UsdPhysics.MaterialAPI.Apply(prim)
        for name, value in (("physics:staticFriction", static),
                            ("physics:dynamicFriction", dynamic)):
            attr = prim.GetAttribute(name)
            if not attr:
                attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)
            attr.Set(float(value))
        return True

    def apply(self):
        if self._syncing:
            return
        w = self.wheel_model
        w.tau = max(0.0, self._num("tau"))
        w.set_delay_steps(int(round(self._num("delay_steps"))))
        w.gain = self._num("gain")
        w.deadband = self._num("deadband")

        n = len(self.wheel_ids)
        damping = max(0.0, self._num("damping"))
        effort = max(0.0, self._num("effort"))
        try:
            self.robot.write_joint_damping_to_sim_index(
                damping=torch.full((1, n), damping, device=self.device),
                joint_ids=self.wheel_ids)
            self.robot.write_joint_effort_limit_to_sim_index(
                limits=torch.full((1, n), effort, device=self.device),
                joint_ids=self.wheel_ids)
        except Exception as exc:
            self.status.text = f"actuator write failed: {exc}"
            return

        missing = [name for name, path in self.fric_paths.items()
                   if not self._set_friction(path, self._num(f"{name}_s"),
                                             self._num(f"{name}_d"))]
        if missing:
            self.status.text = f"live · friction missing: {', '.join(missing)}"
        else:
            self.status.text = "live edit (not saved)"

    def zero(self):
        self._set_all({key: 0.0 for key in self.start})
        self.apply()
        self.status.text = "0 set (not saved)"

    def reset(self):
        self._set_all(self.start)
        self.apply()
        self.status.text = "reset to loaded values"

    def toggle(self):
        self.window.visible = not self.window.visible


def main():
    headless = not getattr(args_cli, "visualizer", None) or "kit" not in (args_cli.visualizer or [])
    seed = args_cli.seed if args_cli.seed is not None else random.randrange(1_000_000)
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))
    rooms = build_world(args_cli.rooms, args_cli.size_min, seed,
                        size_range=(args_cli.size_min, args_cli.size_max), condition="floor")
    set_camera(sim, rooms[0].origin if rooms else (0, 0, 0))
    scene, robot, _ = spawn_robot(condition="floor")
    sim.reset()
    settle(scene, robot, sim)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    dev = robot.device
    r = torch.tensor([spec.WHEEL_RADIUS], device=dev)
    sep = torch.tensor([spec.WHEEL_SEP], device=dev)
    wheel_model = WheelLag(dt=POLICY_DT, condition="floor")

    room_idx = 0
    place_robot(robot, rooms[room_idx].origin if rooms else (0, 0, 0), 0.0, wheel_ids)
    set_camera(sim, rooms[room_idx].origin if rooms else (0, 0, 0))

    def show_room():
        if rooms:
            rm = rooms[room_idx]
            print(f"[01] seed {seed} · 방 {room_idx + 1}/{len(rooms)} · "
                  f"{rm.size:.1f} m · 장애물 {rm.obstacles}개")

    show_room()
    print(f"[01] calib: {calib_desc('floor')}")
    print("[01] W/↑ 전진 · S/↓ 후진 · A/← 좌회전 · D/→ 우회전 · Shift 느리게 · X 정지 · R 리셋 · N 다음 방 · C 보정값")

    kb = None if headless else Keyboard()
    hud = None
    if kb is not None:
        try:
            hud = CalibHUD("floor", wheel_model, robot, wheel_ids, scene.stage, dev)
        except Exception as exc:
            print(f"[01] calib HUD 없이 진행: {exc}")
    target = (0.0, 0.0)
    prev_keys = set()
    reset_latch = next_latch = hud_latch = False
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
                if "C" in p and not hud_latch and hud is not None:
                    hud.toggle()
                hud_latch = "C" in p
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
