"""S5 — sim 계단응답 기록 (real과 같은 CSV 포맷).

프로파일 (20 Hz, 9 s):
  0~1 s  정지 (계단 기준점)
  1~3 s  v=0.15, w=0
  3~5 s  v=0.25, w=0
  5~6 s  정지
  6~8 s  v=0, w=+1.0
  8~9 s  정지

실행 (창 없이, 빠름):
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./steps/run.sh s05_step_response.py --viz none

창 보면서 실시간:
  ./steps/run.sh s05_step_response.py --realtime

출력: steps/artifacts/sim_step.csv
  t, lin_raw, ang_raw, lin_out, ang_out, odom_x, odom_y, odom_yaw, v_enc, w_enc, w_gyro, label
  (real 쪽 eval_recorder와 같은 열 — odom은 실제 드라이버처럼 기준 r/sep로 적분, 정답 pose 아님)
"""
import argparse
import csv
import math
import os
import random
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="S5: sim 계단응답 기록")
parser.add_argument("--out", default=None, help="기본: artifacts/sim_step[_air|_stand].csv")
parser.add_argument("--airborne", action="store_true", help="root 고정 + 바퀴 공중 (무부하)")
parser.add_argument("--stand", action="store_true", help="배 밑 지지 박스 위에 올림 (바퀴 공중, 실제 셋업과 동일)")
parser.add_argument("--stand-height", type=float, default=0.06, help="지지 박스 높이 [m]")
parser.add_argument("--room", type=float, default=4.0)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--speed1", type=float, default=0.15)
parser.add_argument("--speed2", type=float, default=0.25)
parser.add_argument("--turn", type=float, default=1.0)
parser.add_argument("--gyro-noise", type=float, default=0.0, help="자이로 잡음 std [rad/s] (기본 0 = 깨끗한 비교)")
parser.add_argument("--realtime", action="store_true", help="20 Hz 실시간으로 (창으로 볼 때)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from scene import build_world, set_camera, settle, spawn_robot  # noqa: E402
from tasks.common import robot_spec as spec  # noqa: E402
from tasks.rl_correct.rl_correct_env import wheel_targets  # noqa: E402

DT = 1.0 / 120.0
POLICY_DT = 1.0 / spec.CONTROL_HZ      # 0.05 s
SUBSTEPS = round(POLICY_DT / DT)       # 6
SEGMENTS = (  # (시작, 끝, v, w)
    (0.0, 1.0, 0.0, 0.0),
    (1.0, 3.0, "speed1", 0.0),
    (3.0, 5.0, "speed2", 0.0),
    (5.0, 6.0, 0.0, 0.0),
    (6.0, 8.0, 0.0, "turn"),
    (8.0, 9.0, 0.0, 0.0),
)
TOTAL = 9.0


def cmd_at(t: float) -> tuple[float, float]:
    for start, end, v, w in SEGMENTS:
        if t < end:
            v = args_cli.speed1 if v == "speed1" else args_cli.speed2 if v == "speed2" else v
            w = args_cli.turn if w == "turn" else w
            return float(v), float(w)
    return 0.0, 0.0


def main():
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))
    set_camera(sim)
    build_world(1 if args_cli.room > 0 else 0, args_cli.room, args_cli.seed)
    stand_h = args_cli.stand_height if args_cli.stand else 0.0
    if args_cli.stand:
        # 배 밑 지지 박스: 폭 0.15(y) < 바퀴 안쪽(0.136) → 바퀴는 박스에 안 닿음
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0)
        cfg = sim_utils.CuboidCfg(
            size=(0.20, 0.15, stand_h),
            collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
            physics_material=mat,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.30, 0.15)),
        )
        cfg.func("/World/stand", cfg, translation=(-0.064, 0.0, stand_h / 2.0))

    scene, robot, _ = spawn_robot(
        airborne=args_cli.airborne,
        init_z=(stand_h + 0.03) if args_cli.stand else None,
    )
    sim.reset()
    z = settle(scene, robot, sim)
    tag = "sim_stand" if args_cli.stand else ("sim_air" if args_cli.airborne else "sim")
    out = args_cli.out or str(HERE / "artifacts" / f"{tag}_step.csv")
    if not os.path.isabs(out):
        out = str(HERE.parent / out)   # isaac/ 기준 (run.sh가 IsaacLab으로 cd하므로)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    dev = robot.device
    r = torch.tensor([spec.WHEEL_RADIUS], device=dev)
    sep = torch.tensor([spec.WHEEL_SEP], device=dev)
    mode = (f" · STAND h={stand_h:.2f} m (바퀴 공중)" if args_cli.stand
            else (" · AIRBORNE (w_gyro≈0, w_enc=명령)" if args_cli.airborne else ""))
    print(f"[s5] base 높이 {z:.3f} m · sim {TOTAL:.0f} s · {spec.CONTROL_HZ:.0f} Hz 기록{mode}")

    rows = []
    ox, oy, oyaw = 0.0, 0.0, 0.0
    n = int(round(TOTAL / POLICY_DT))
    for k in range(n):
        frame_start = time.perf_counter()
        t = k * POLICY_DT
        v, w = cmd_at(t)
        wheels = wheel_targets(torch.tensor([v], device=dev), torch.tensor([w], device=dev),
                               r, sep, spec.MAX_WHEEL)
        robot.set_joint_velocity_target(wheels, joint_ids=wheel_ids)
        scene.write_data_to_sim()
        for _ in range(SUBSTEPS):
            sim.step(render=False)
        scene.update(POLICY_DT)

        jv = robot.data.joint_vel.torch[0, wheel_ids].tolist()
        v_enc = spec.WHEEL_RADIUS * (jv[0] + jv[1]) / 2.0
        w_enc = spec.WHEEL_RADIUS * (jv[1] - jv[0]) / spec.WHEEL_SEP
        w_gyro = robot.data.root_ang_vel_b.torch[0, 2].item()
        if args_cli.gyro_noise > 0:
            w_gyro += random.gauss(0.0, args_cli.gyro_noise)
        oyaw += w_enc * POLICY_DT
        ox += v_enc * math.cos(oyaw) * POLICY_DT
        oy += v_enc * math.sin(oyaw) * POLICY_DT
        rows.append([round(t, 3), v, w, v, w, ox, oy, oyaw, v_enc, w_enc, w_gyro, tag])

        sim.render()
        if args_cli.realtime:
            rest = POLICY_DT - (time.perf_counter() - frame_start)
            if rest > 0:
                time.sleep(rest)

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(["t", "lin_raw", "ang_raw", "lin_out", "ang_out",
                     "odom_x", "odom_y", "odom_yaw", "v_enc", "w_enc", "w_gyro", "label"])
        cw.writerows(rows)

    moving = [r for r in rows if r[1] != 0.0 or r[2] != 0.0]
    rms_v = math.sqrt(sum((r[1] - r[8]) ** 2 for r in moving) / max(len(moving), 1))
    rms_w = math.sqrt(sum((r[2] - r[10]) ** 2 for r in moving) / max(len(moving), 1))
    x, y = robot.data.root_pos_w.torch[0, :2].tolist()
    yaw = robot.data.heading_w.torch[0].item()
    print(f"[s5] rows {len(rows)} · rms_v {rms_v:.4f} m/s · rms_w {rms_w:.4f} rad/s")
    print(f"[s5] true pose x={x:+.2f} y={y:+.2f} yaw={yaw:+.2f} · odom x={ox:+.2f} y={oy:+.2f} yaw={oyaw:+.2f}")
    print(f"[s5] → {out}")
    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
