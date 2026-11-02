"""02 — 센서 싱크 (sim): 같은 명령에서 v_enc · w_enc · w_gyro 기록.

프로파일 (20 Hz, 기본 약 54 s):
  2 s 정지 → 왼쪽 사각형(반시계) → 180° 턴 → 같은 사각형 역주행
  → 2 s 정지. 각 변: 0.5 m 전진 → 0.25 s 정지 → 제자리 90도 회전 → 0.25 s 정지.

실행:
  # 바닥 (기본: 장애물 없는 빈 바닥) — 기록
  ./run-calibration.sh sensor-sync-sim --viz none

  # 눈으로 보기 (창 닫을 때까지 반복)
  ./run-calibration.sh sensor-sync-sim --watch

  # 보정은 --fit으로 준 값만 (없으면 raw plant: gain 1 · delay 0 · tau 0 · deadband 0)
  ./run-calibration.sh sensor-sync-sim --viz none --fit <real>/fit.json[,<sim>/fit.json]

출력: artifacts/calibration/<시각>-02-sim/ (raw.csv · fit.json · report.png, --fit이면 base-fit.json)
CSV: /joint_states(raw wheel rad/s) + /odom(v_enc,w_enc) + /imu(w_gyro)
real: ./run-calibration.sh sensor-sync-real --label real_floor
비교: ./run-calibration.sh plot-compare <sim.csv> <real.csv>
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ISAAC))

from isaaclab.app import AppLauncher  # noqa: E402

# pxr/isaaclab은 AppLauncher(args) 뒤에 import한다 (먼저 import하면 SimulationApp이 깨짐).
from calibration.sensor.Sensor import Sensor  # noqa: E402
from urdf import robot_spec as spec  # noqa: E402

DT = 1 / 120.0
POLICY_DT = 1 / spec.CONTROL_HZ
SUBSTEPS = round(POLICY_DT / DT)


class DefaultSensorSim(Sensor):
    """sim 실행기 — 관절 타깥 + 물리 스텝으로 명령을 보내고 시뮬 상태에서 행을 만든다."""

    TAG = "02-sim"

    def __init__(self, sim, scene, robot, wheel_ids, wheel_model, label: str):
        super().__init__()
        self.sim = sim
        self.scene = scene
        self.robot = robot
        self.wheel_ids = wheel_ids
        self.wheel_model = wheel_model
        self.label = label
        self.dev = robot.device
        self.r = torch.tensor([spec.WHEEL_RADIUS], device=self.dev)
        self.sep = torch.tensor([spec.WHEEL_SEP], device=self.dev)
        self.ox = self.oy = self.oyaw = 0.0

    def send(self, v: float, w: float) -> None:
        wheels = self.wheel_model.step(v, w, wheel_targets, self.r, self.sep,
                                       spec.MAX_WHEEL, self.dev)
        self.robot.set_joint_velocity_target(wheels, joint_ids=self.wheel_ids)
        self.scene.write_data_to_sim()
        for _ in range(SUBSTEPS):
            self.sim.step(render=False)
        self.scene.update(POLICY_DT)

    def collect(self, t: float, v: float, w: float) -> list:
        jv = self.robot.data.joint_vel.torch[0, self.wheel_ids].tolist()
        jp = self.robot.data.joint_pos.torch[0, self.wheel_ids].tolist()
        v_enc = spec.WHEEL_RADIUS * (jv[0] + jv[1]) / 2.0
        w_enc = spec.WHEEL_RADIUS * (jv[1] - jv[0]) / spec.WHEEL_SEP
        w_gyro = self.robot.data.root_ang_vel_b.torch[0, 2].item()
        self.oyaw += w_enc * POLICY_DT
        self.ox += v_enc * math.cos(self.oyaw) * POLICY_DT
        self.oy += v_enc * math.sin(self.oyaw) * POLICY_DT
        # sim은 단일 시계로 돌므로 msg 시각 = 기록 시각 (age 0 기준선)
        return [round(t, 3), v, w, v, w, self.ox, self.oy, self.oyaw,
                v_enc, w_enc, w_gyro, jv[0], jv[1], jp[0], jp[1], self.label,
                t, t, t, t]

    @classmethod
    def main(cls, argv=None):
        parser = argparse.ArgumentParser(description="02: sim 센서 싱크 (바닥)")
        parser.add_argument("--room", type=float, default=0.0,
                            help="방 크기 [m] (0=빈 바닥, 장애물 없음). 센서 싱크는 0 권장")
        parser.add_argument("--seed", type=int, default=0)
        parser.add_argument("--speed", type=float, default=0.15,
                            help="기준 전진 속도 [m/s] (변마다 1.0/0.67/1.33/0.8 배)")
        parser.add_argument("--turn", type=float, default=1.0)
        parser.add_argument("--side", type=float, default=0.5, help="정사각형 한 변 [m]")
        parser.add_argument("--out", default=None,
                            help="기본: artifacts/calibration/<시각>-<TAG>/raw.csv")
        parser.add_argument("--fit", default=None,
                            help="입력 보정: real/fit.json 또는 real/fit.json,sim/fit.json (차이) → WheelLag. "
                                 "없으면 raw plant (숨은 보정 파일은 안 읽는다)")
        parser.add_argument("--damping", type=float, default=None,
                            help="바퀴 모터 damping 덮어쓰기 (측정용 — 기본: ActuatorParam). fit.json의 plant에 기록")
        parser.add_argument("--effort", type=float, default=None,
                            help="바퀴 모터 effort limit 덮어쓰기 [N·m] (측정용)")
        parser.add_argument("--velocity-limit", type=float, default=None,
                            help="바퀴 모터 velocity limit 덮어쓰기 [rad/s] (측정용)")
        parser.add_argument("--watch", action="store_true", help="1회 기록 후 창을 닫을 때까지 같은 장면 반복")
        AppLauncher.add_app_launcher_args(parser)
        args = parser.parse_args(argv)

        app_launcher = AppLauncher(args)
        simulation_app = app_launcher.app

        # AppLauncher가 pxr/physx를 초기화한 뒤에 런타임 모듈을 import한다.
        global torch, SimulationCfg, SimulationContext, Calibrate, DefaultScene
        global DefaultCalibExcitation, ActuatorParam, SimParam, WheelLag, wheel_targets
        import torch
        from isaaclab.sim import SimulationCfg, SimulationContext
        from calibration.Calibrate import Calibrate
        from calibration.scene.DefaultScene import DefaultScene
        from calibration.excitation.DefaultCalibExcitation import DefaultCalibExcitation
        from calibration.parameter.ActuatorParam import ActuatorParam
        from calibration.parameter.SimParam import SimParam
        from calibration.parameter.WheelLag import WheelLag
        from training.rl_correction_wheel_drift.task.rl_correct_env import wheel_targets

        try:
            cls._run(args, simulation_app)
        finally:
            simulation_app.close()

    @classmethod
    def _run(cls, args, simulation_app):
        headless = not getattr(args, "visualizer", None) or "kit" not in (args.visualizer or [])
        # 보정은 사용자가 --fit으로 준 값만. 저장된 sim_calib_*.json은 읽지 않는다 (raw plant 기준).
        condition = "raw"
        excitation = DefaultCalibExcitation()
        segs = excitation.make_profile(args.speed, args.turn, args.side)
        total = excitation.total(segs)
        sim = SimulationContext(SimulationCfg(dt=DT, device=args.device))
        act = ActuatorParam()
        if args.damping is not None:
            act.damping = args.damping
        if args.effort is not None:
            act.effort = args.effort
        if args.velocity_limit is not None:
            act.velocity_limit = args.velocity_limit
        world = DefaultScene(condition=condition, actuator=act)
        rooms = world.build(1 if args.room > 0 else 0, args.room, args.seed)
        world.set_camera(sim, rooms[0].origin if rooms else (0.0, 0.0, 0.0))
        scene, robot, _ = world.spawn()
        sim.reset()
        z = world.settle(sim)
        wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
        params = SimParam.raw()
        params.actuator = act
        base_fit = Calibrate.load_fit(args.fit) if args.fit else None
        if base_fit is not None:
            params.wheel = SimParam.from_dict(base_fit).wheel
        wheel_model = WheelLag(params.wheel, dt=POLICY_DT)
        tag = "sim_sensor_floor"
        out = args.out
        if out and not os.path.isabs(out):
            out = str(ISAAC / out)   # isaac/ 기준 (예: artifacts/...)
        env = "empty floor (no obstacles)" if args.room <= 0 else f"room {args.room:.1f} m"
        mode = f"FIT {', '.join(base_fit['source'])}" if base_fit else "RAW (no --fit)"
        print(f"[02-sim] base 높이 {z:.3f} m · {tag} · {mode} · {env}")
        print(f"[02-sim] calib: {params.describe()}")
        sync = cls(sim, scene, robot, wheel_ids, wheel_model, label=tag)
        speeds_txt = sync.move_speeds_text(segs)
        print(f"[02-sim] profile: side={args.side:.2f} m · {total:.1f}s · "
              f"speeds {speeds_txt} m/s · left square → 180° turn → reverse")
        cal = Calibrate(sync)
        cal.run(segs, total, out, POLICY_DT, base_fit=base_fit,
                plant={"actuator_damping": act.damping, "actuator_effort": act.effort,
                       "actuator_velocity_limit": act.velocity_limit})

        if args.watch and not headless:
            print("[02-sim] WATCH — 창을 닫을 때까지 반복 (left square → right square)")
            sim_t = 0.0
            next_cmd = 0.0
            while simulation_app.is_running():
                if sim_t >= next_cmd:
                    next_cmd += POLICY_DT
                    v, w = excitation.cmd_at(sim_t % total, segs)
                    sync.send(v, w)
                for _ in range(SUBSTEPS):
                    sim.step(render=False)
                scene.update(POLICY_DT)
                sim.render()
                sim_t += POLICY_DT
                time.sleep(POLICY_DT)
        sim.stop()


if __name__ == "__main__":
    DefaultSensorSim.main()
