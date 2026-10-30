"""02 — 센서 싱크 (sim): 같은 명령에서 v_enc · w_enc · w_gyro 기록.

프로파일 (20 Hz, 기본 약 47 s):
  2 s 정지 → 왼쪽 사각형(반시계) → 0.5 s 정지 → 오른쪽 사각형(시계) → 2 s 정지
  각 변: 0.5 m 전진 → 0.25 s 정지 → 제자리 90도 회전 → 0.25 s 정지

실행:
  # 공중 (배 밑 지지 박스, 바퀴 공중) — 실제 로봇을 박스 위에 올린 상태와 비교
  ./steps/02_sensor_sync.sh --viz none --stand

  # 눈으로 보기 (박스 위, 창 닫을 때까지 반복)
  ./steps/02_sensor_sync.sh --stand --watch

  # 바닥
  ./steps/02_sensor_sync.sh --viz none

출력: steps/artifacts/sim_sensor_stand.csv 또는 sim_sensor_floor.csv
CSV: /joint_states(raw wheel rad/s) + /odom(v_enc,w_enc) + /imu(w_gyro)
real: ./steps/02_sensor_sync_real.sh --label real_floor
비교: ./steps/plot_compare.sh <sim.csv> <real.csv>
"""
import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parent
sys.path.insert(0, str(HERE))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="02: sim 센서 싱크")
parser.add_argument("--stand", action="store_true", help="배 밑 지지 박스 위 (바퀴 공중)")
parser.add_argument("--stand-height", type=float, default=0.06, help="지지 박스 높이 [m]")
parser.add_argument("--room", type=float, default=4.0)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--speed", type=float, default=0.15)
parser.add_argument("--turn", type=float, default=1.0)
parser.add_argument("--side", type=float, default=0.5, help="정사각형 한 변 [m]")
parser.add_argument("--out", default=None, help="기본: steps/artifacts/sim_sensor_[stand|floor].csv")
parser.add_argument("--watch", action="store_true", help="1회 기록 후 창을 닫을 때까지 같은 장면 반복")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from scene import build_world, set_camera, settle, spawn_robot  # noqa: E402
from square_profile import cmd_at, make_profile, total as profile_total  # noqa: E402
from tasks.common import robot_spec as spec  # noqa: E402
from tasks.common.sim_params import WheelLag, describe as calib_desc  # noqa: E402
from rl.wheel_drift.task.rl_correct_env import wheel_targets  # noqa: E402

DT = 1 / 120.0
POLICY_DT = 1 / spec.CONTROL_HZ
SUBSTEPS = round(POLICY_DT / DT)


def main():
    headless = not getattr(args_cli, "visualizer", None) or "kit" not in (args_cli.visualizer or [])
    condition = "stand" if args_cli.stand else "floor"
    segs = make_profile(args_cli.speed, args_cli.turn, args_cli.side)
    total = profile_total(segs)
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))
    stand_h = args_cli.stand_height if args_cli.stand else 0.0
    room = build_world(1 if args_cli.room > 0 else 0, args_cli.room, args_cli.seed,
                       condition=condition)
    if args_cli.stand:
        # 박스 위 로봇을 옆에서: 배 밑 박스와 공중 바퀴가 보이게
        sim.set_camera_view(eye=(0.65, -0.65, 0.45), target=(-0.064, 0.0, 0.08))
    else:
        set_camera(sim, room[0].origin if room else (0, 0, 0))
    if args_cli.stand:
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0)
        box = sim_utils.CuboidCfg(
            size=(0.20, 0.15, stand_h),
            collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
            physics_material=mat,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.30, 0.15)),
        )
        box.func("/World/stand", box, translation=(-0.064, 0.0, stand_h / 2.0))
    scene, robot, _ = spawn_robot(init_z=(stand_h + 0.03) if args_cli.stand else None,
                                  condition=condition)
    sim.reset()
    z = settle(scene, robot, sim)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    dev = robot.device
    r = torch.tensor([spec.WHEEL_RADIUS], device=dev)
    sep = torch.tensor([spec.WHEEL_SEP], device=dev)
    wheel_model = WheelLag(dt=POLICY_DT, condition=condition)
    tag = "sim_sensor_stand" if args_cli.stand else "sim_sensor_floor"
    out = args_cli.out or str(HERE / "artifacts" / f"{tag}.csv")
    if not os.path.isabs(out):
        out = str(HERE.parent / out)   # isaac/ 기준 (예: steps/artifacts/...)
    print(f"[02-sim] base 높이 {z:.3f} m · {tag} · {'STAND' if args_cli.stand else 'FLOOR'}")
    print(f"[02-sim] calib: {calib_desc(condition)}")
    print(f"[02-sim] profile: side={args_cli.side:.2f} m · {total:.1f}s · "
          f"left/right square")

    rows = []
    ox = oy = oyaw = 0.0
    n = int(round(total / POLICY_DT))
    for k in range(n):
        t = k * POLICY_DT
        v, w = cmd_at(t, segs)
        wheels = wheel_model.step(v, w, wheel_targets, r, sep, spec.MAX_WHEEL, dev)
        robot.set_joint_velocity_target(wheels, joint_ids=wheel_ids)
        scene.write_data_to_sim()
        for _ in range(SUBSTEPS):
            sim.step(render=False)
        scene.update(POLICY_DT)

        jv = robot.data.joint_vel.torch[0, wheel_ids].tolist()
        jp = robot.data.joint_pos.torch[0, wheel_ids].tolist()
        v_enc = spec.WHEEL_RADIUS * (jv[0] + jv[1]) / 2.0
        w_enc = spec.WHEEL_RADIUS * (jv[1] - jv[0]) / spec.WHEEL_SEP
        w_gyro = robot.data.root_ang_vel_b.torch[0, 2].item()
        oyaw += w_enc * POLICY_DT
        ox += v_enc * math.cos(oyaw) * POLICY_DT
        oy += v_enc * math.sin(oyaw) * POLICY_DT
        rows.append([round(t, 3), v, w, v, w, ox, oy, oyaw, v_enc, w_enc, w_gyro,
                     jv[0], jv[1], jp[0], jp[1], tag])

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(["t", "lin_raw", "ang_raw", "lin_out", "ang_out",
                     "odom_x", "odom_y", "odom_yaw", "v_enc", "w_enc", "w_gyro",
                     "wheel_l_vel", "wheel_r_vel", "wheel_l_pos", "wheel_r_pos", "label"])
        cw.writerows(rows)

    # 구간 평균 — 첫 왼쪽/오른쪽 변과 회전
    def average(t0, t1, skip=0.0):
        part = [r for r in rows if t0 + skip <= r[0] < t1]
        if not part:
            part = [r for r in rows if t0 <= r[0] < t1]
        return tuple(sum(r[i] for r in part) / len(part) for i in (8, 9, 10, 11, 12))

    def first(label):
        return next(s for s in segs if s[4] == label)

    for name in ("left", "right"):
        move, turn = first(f"move_{name}"), first(f"turn_{name}")
        vf, wf, gf, lf, rf = average(move[0], move[1], skip=0.5)
        vt, wt, gt, lt, rt = average(turn[0], turn[1], skip=0.3)
        print(f"[02-sim] {name.upper():5s} move  v_enc={vf:+.3f} w_enc={wf:+.3f} "
              f"w_gyro={gf:+.3f} | 바퀴 wl={lf:+.2f} wr={rf:+.2f} rad/s")
        print(f"[02-sim] {name.upper():5s} turn  v_enc={vt:+.3f} w_enc={wt:+.3f} "
              f"w_gyro={gt:+.3f} | 바퀴 wl={lt:+.2f} wr={rt:+.2f} rad/s")
    print(f"[02-sim] → {out}")

    if args_cli.watch and not headless:
        print("[02-sim] WATCH — 창을 닫을 때까지 반복 (left square → right square)")
        sim_t = 0.0
        next_cmd = 0.0
        while simulation_app.is_running():
            if sim_t >= next_cmd:
                next_cmd += POLICY_DT
                v, w = cmd_at(sim_t % total, segs)
                wheels = wheel_model.step(v, w, wheel_targets, r, sep,
                                          spec.MAX_WHEEL, dev)
                robot.set_joint_velocity_target(wheels, joint_ids=wheel_ids)
                scene.write_data_to_sim()
                scene.update(POLICY_DT)
            for _ in range(SUBSTEPS):
                sim.step(render=False)
            scene.update(POLICY_DT)
            sim.render()
            sim_t += POLICY_DT
            time.sleep(POLICY_DT)
    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
