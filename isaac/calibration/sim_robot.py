"""calibration sim — SimRobot(robots/sim_robot.py)에 명령(기본 사각형)을 CalibrationEnv(calibration/env.py = envs/default: 순수 sim, 보정 없음)에 넣고 v_enc · w_enc · w_gyro를 기록 → raw.csv · fit.json · report.png.

  ./run-calibration.sh sim --viz none                  # 순수 sim 측정 → real과 비교해 차이를 구한다
창(--watch): 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 노란 점 = 목표 경로. 기록 후 창 닫을 때까지 같은 경로 반복.
출력: artifacts/calibration/<시각>-<label>/ (--label 없으면 02-sim) · --out으로 정확한 경로.
"""
from __future__ import annotations

import argparse
import os
import time

from isaaclab.app import AppLauncher

from calibration.record import ISAAC, add_command_args, make_command, record, run_path

TAG = "02-sim"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="sim 측정 (calibration env)")
    add_command_args(p)
    p.add_argument("--room", type=float, default=0.0, help="방 크기 [m] (0 = 장애물 없는 빈 바닥, 측정은 0 권장)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=None, help="정확한 raw.csv 경로 (isaac/ 기준 상대경로 가능)")
    p.add_argument("--label", default=None, help="폴더 이름 <시각>-<label> · CSV label (기본 02-sim / sim_sensor_floor)")
    p.add_argument("--watch", action="store_true", help="기록 후 창 닫을 때까지 같은 경로 반복")
    AppLauncher.add_app_launcher_args(p)
    args = p.parse_args(argv)

    # sim 띄우기 전에 실패하게
    cmd = make_command(args)
    out = args.out and (args.out if os.path.isabs(args.out) else str(ISAAC / args.out))
    out = out or run_path(args.label or TAG)

    app = AppLauncher(args).app
    try:
        _run(args, app, cmd, out)
    except BaseException:
        import traceback
        traceback.print_exc()   # app.close()가 프로세스를 바로 끝내 예외가 안 보임 → 먼저 출력
        raise
    finally:
        app.close()
    return 0


def _run(args, app, cmd, out):
    import torch

    from calibration.env import CalibrationEnv
    from calibration.env_cfg import CalibrationEnvCfg
    from envs.rooms.env import RoomsEnv
    from envs.rooms.env_cfg import RoomsEnvCfg

    headless = "kit" not in (getattr(args, "visualizer", None) or [])
    if args.room > 0:   # 방 1개 (측정은 빈 바닥 권장)
        cfg, env_cls = RoomsEnvCfg(), RoomsEnv
        cfg.rooms, cfg.room_size, cfg.room_seed = 1, (args.room, args.room), args.seed
    else:
        cfg, env_cls = CalibrationEnvCfg(), CalibrationEnv
    cfg.sim.device = args.device
    cfg.debug_vis = not headless
    env = env_cls(cfg)
    env.reset()
    zero = torch.zeros(1, 2, device=env.device)
    for _ in range(10):   # 0.5 s — 스폰 높이에서 떨어져 바퀴·캐스터로 서게
        env.step(zero)
    p0 = env.robot.data.root_pos_w.torch[0]
    if not headless and cmd.path():
        env.set_path(cmd.path())   # 노란 점 = 목표 경로를 로봇 시작 자세로
    from robots.sim_robot import SimRobot

    robot, label = SimRobot(env), args.label or "sim_sensor_floor"
    print(f"[{TAG}] 높이 {float(p0[2]):.3f} m · {label} · 순수 sim (보정 없음) · "
          f"{'room %.1f m' % args.room if args.room > 0 else 'empty floor'}")
    print(f"[{TAG}] {args.cmd}: side={args.side:.2f} m · {cmd.total:.1f}s · speeds {getattr(cmd, 'speeds_text', lambda: '-')()} m/s")
    record(robot, cmd, out, TAG, label)

    if args.watch and not headless:
        print(f"[{TAG}] WATCH — 창 닫을 때까지 반복")
        k = 0
        while app.is_running():
            robot.write(*cmd((k * robot.dt) % cmd.total))
            k += 1
            time.sleep(robot.dt)
    env.close()


if __name__ == "__main__":
    raise SystemExit(main())
