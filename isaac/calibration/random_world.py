"""random-world — 랜덤 방(벽 + 장애물) 여러 개 + 로봇 키보드 주행 (calibration env, 창 필요).

  ./run-calibration.sh random-world [--fit <real>/fit.json[,<sim>/fit.json]] [--seed 7] [--rooms 8]
키: W/A/S/D(화살표) · Shift 느리게 · X 정지 · R 방 가운데로 · N 다음 방 · C 보정값 창 (숫자 고치면 바로 반영, 저장 안 함)
"""
from __future__ import annotations

import argparse
import random
import time

from isaaclab.app import AppLauncher

from calibration.fit import describe, load_fit, wheel_of

TAG = "random-world"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="random-world: 랜덤 방 + 로봇 키보드 주행")
    p.add_argument("--rooms", type=int, default=8, help="미리 만들 방 수 (N으로 순간이동)")
    p.add_argument("--seed", type=int, default=None, help="랜덤 시드 (없으면 매번 새로)")
    p.add_argument("--size-min", type=float, default=3.5, help="방 최소 한 변 [m]")
    p.add_argument("--size-max", type=float, default=6.0, help="방 최대 한 변 [m]")
    p.add_argument("--speed", type=float, default=0.15, help="전진 [m/s]")
    p.add_argument("--turn", type=float, default=1.0, help="회전 [rad/s]")
    p.add_argument("--seconds", type=float, default=0.0, help="0 = 창 닫을 때까지")
    p.add_argument("--fit", default=None, help="입력 보정 real/fit.json 또는 real,sim (차이). 없으면 raw")
    AppLauncher.add_app_launcher_args(p)
    args = p.parse_args(argv)
    base_fit = load_fit(args.fit) if args.fit else None   # sim 띄우기 전에 실패하게
    app = AppLauncher(args).app
    try:
        _run(args, app, base_fit)
    except BaseException:
        import traceback
        traceback.print_exc()   # app.close()가 프로세스를 바로 끝내 예외가 안 보임 → 먼저 출력
        raise
    finally:
        app.close()
    return 0


def _run(args, app, base_fit):
    import torch

    from calibration.calib_env import CalibEnv
    from calibration.calib_env_cfg import RoomsEnvCfg, WheelModelCfg

    headless = "kit" not in (getattr(args, "visualizer", None) or [])
    cfg = RoomsEnvCfg()
    cfg.sim.device = args.device
    cfg.rooms, cfg.room_size = args.rooms, (args.size_min, args.size_max)
    cfg.room_seed = args.seed if args.seed is not None else random.randrange(1_000_000)
    if base_fit is not None:
        cfg.wheel = WheelModelCfg(**wheel_of(base_fit))
    env = CalibEnv(cfg)
    env.reset()
    rooms, idx = env.rooms, 0

    def go(i):
        env.place(rooms[i]["origin"])
        env.look_at(rooms[i]["origin"])
        rm = rooms[i]
        print(f"[{TAG}] seed {cfg.room_seed} · 방 {i + 1}/{len(rooms)} · {rm['size']:.1f} m · 장애물 {rm['obstacles']}개")

    go(idx)
    print(f"[{TAG}] wheel: {describe(cfg.wheel.to_dict())}")
    print(f"[{TAG}] W/A/S/D(화살표) · Shift 느리게 · X 정지 · R 리셋 · N 다음 방 · C 보정값 창")
    kb = hud = None
    if not headless:
        from calibration import keyboard as kb
        try:
            hud = CalibHUD(env, base_fit)
        except Exception as exc:
            print(f"[{TAG}] 보정값 창 없이 진행: {exc}")
    prev, t0 = set(), time.monotonic()
    while app.is_running() and not (args.seconds > 0 and time.monotonic() - t0 >= args.seconds):
        t = time.monotonic()
        v = w = 0.0
        if kb is not None:
            keys = kb.pressed()
            new, prev = keys - prev, keys
            v, w = kb.drive_cmd(keys, args.speed, args.turn, slow=0.4)
            if "C" in new and hud is not None:
                hud.toggle()
            if "R" in new:
                go(idx)
            if "N" in new and rooms:
                idx = (idx + 1) % len(rooms)
                go(idx)
        env.step(torch.tensor([[v, w]], device=env.device))
        time.sleep(max(0.0, env.policy_dt - (time.monotonic() - t)))   # 실시간
    env.close()


class CalibHUD:
    """실행 중 바퀴 모델 · 마찰을 고치는 창 (omni.ui). 숫자 수정 즉시 env에 반영 · 0 set · reset(열 때 값)."""

    LABEL = {"font_size": 13, "color": 0xFF9FB4C7}
    HEAD = {"font_size": 12, "color": 0xFFF0C040}
    FRICTION = {"ground": "/World/ground/physicsMaterial", "wheel": "/World/Materials/wheel",
                "caster": "/World/Materials/caster", "obstacle": "/World/Materials/obstacle"}

    def __init__(self, env, base_fit):
        import omni.ui as ui
        from assets.tb3_waffle_pi import CASTER_FRICTION, GROUND_FRICTION, WHEEL_FRICTION

        self.env, self.ui, self.models, self._sync = env, ui, {}, False
        w, c = env.cfg.wheel, env.cfg
        fr = {"ground": GROUND_FRICTION, "wheel": WHEEL_FRICTION, "caster": CASTER_FRICTION, "obstacle": c.obstacle_friction}
        self.start = {"tau_s": w.tau_s, "delay_steps": w.delay_steps, "gain_l": w.gain_l, "gain_r": w.gain_r,
                      "deadband": w.deadband} | {f"{k}_{i}": v[i] for k, v in fr.items() for i in (0, 1)}
        self.window = ui.Window("SIM CALIBRATION", width=450, height=470)
        with self.window.frame:
            with ui.VStack(spacing=4):
                ui.Label("WHEEL MODEL", height=20, style=self.HEAD)
                for key, label in (("tau_s", "tau [s]"), ("delay_steps", "dead steps"), ("gain_l", "gain L"),
                                   ("gain_r", "gain R"), ("deadband", "deadband [rad/s]")):
                    with ui.HStack(height=22):
                        ui.Label(label, width=145, style=self.LABEL)
                        self._field(key, 120)
                ui.Label("FRICTION (static / dynamic)", height=20, style=self.HEAD)
                for name in self.FRICTION:
                    with ui.HStack(height=22):
                        ui.Label(name, width=145, style=self.LABEL)
                        self._field(f"{name}_0", 65)
                        self._field(f"{name}_1", 65)
                with ui.HStack(spacing=6, height=30):
                    ui.Button("0 set", clicked_fn=lambda: self._set({k: 0 for k in self.start}), width=110)
                    ui.Button("reset", clicked_fn=lambda: self._set(self.start), width=110)
                src = ", ".join(base_fit["source"]) if base_fit else "raw (no --fit)"
                self.status = ui.Label(f"source: {src}", height=20, style=self.LABEL)

    def _field(self, key, width):
        ui = self.ui
        if key == "delay_steps":
            m = ui.SimpleIntModel(int(self.start[key]))
            ui.IntField(m, width=width)
        else:
            m = ui.SimpleFloatModel(float(self.start[key]))
            ui.FloatField(m, width=width)
        m.add_value_changed_fn(lambda _m: self.apply())
        self.models[key] = m

    def _num(self, key):
        m = self.models[key]
        return m.get_value_as_int() if key == "delay_steps" else m.get_value_as_float()

    def _set(self, values):
        self._sync = True
        try:
            for k, m in self.models.items():
                m.set_value(int(values[k]) if k == "delay_steps" else float(values[k]))
        finally:
            self._sync = False
        self.apply()

    def apply(self):
        if self._sync:
            return
        w = self.env.cfg.wheel
        w.tau_s, w.delay_steps = max(0.0, self._num("tau_s")), max(0, self._num("delay_steps"))
        w.gain_l, w.gain_r, w.deadband = self._num("gain_l"), self._num("gain_r"), self._num("deadband")
        from pxr import Sdf, UsdPhysics

        missing = []
        for name, path in self.FRICTION.items():
            prim = self.env.scene.stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                missing.append(name)
                continue
            if not prim.HasAPI(UsdPhysics.MaterialAPI):
                UsdPhysics.MaterialAPI.Apply(prim)
            for attr, i in (("physics:staticFriction", 0), ("physics:dynamicFriction", 1)):
                a = prim.GetAttribute(attr) or prim.CreateAttribute(attr, Sdf.ValueTypeNames.Float)
                a.Set(float(self._num(f"{name}_{i}")))
        self.status.text = f"live · friction 없음: {', '.join(missing)}" if missing else "live edit (저장 안 함)"

    def toggle(self):
        self.window.visible = not self.window.visible


if __name__ == "__main__":
    raise SystemExit(main())
