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

  # 학습한 정책을 sim에서 play — corrector와 같은 규칙으로 보정하며 같은 경로를 달린다 (실기기 A/B를 sim에서 미리)
  ./run-calibration.sh sensor-sync-sim --watch --fit <real>,<sim> --policy artifacts/real/policy.onnx --label B
  ./run-calibration.sh sensor-sync-sim --viz none --fit <real>,<sim> --label A        # 같은 조건, 보정 없이
  ../ros2_ws/report.py <A>/raw.csv <B>/raw.csv                                         # A/B 비교

창(--watch)에서: 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 노란 점 = 목표 경로(사각형).

출력: artifacts/calibration/<시각>-<label>/ (raw.csv · fit.json · report.png, --fit이면 base-fit.json)
      --label 없으면 <시각>-02-sim, CSV label은 sim_sensor_floor. --out으로 정확한 경로 지정.
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

DT = 1 / spec.PHYSICS_HZ
POLICY_DT = 1 / spec.CONTROL_HZ
SUBSTEPS = round(POLICY_DT / DT)


class DefaultSensorSim(Sensor):
    """sim 실행기 — 관절 타깥 + 물리 스텝으로 명령을 보내고 시뮬 상태에서 행을 만든다."""

    TAG = "02-sim"

    def __init__(self, sim, scene, robot, wheel_ids, wheel_model, label: str,
                 policy=None, markers=None, path_w=None):
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
        self.policy = policy          # onnxruntime 세션 (--policy) — 없으면 입력 그대로
        self.markers = markers        # CommandMarkers (창 볼 때만)
        self.path_w = path_w          # 목표 경로 점 (m, 3)
        self.last_out = (0.0, 0.0)
        self._t = 0.0
        self._last_cmd = (0.0, 0.0)
        self._changed_at = 0.0

    def _sense(self):
        """(바퀴 rad/s 좌우, v_enc, w_enc, w_gyro) — 드라이버 /odom · /imu와 같은 식."""
        jv = self.robot.data.joint_vel.torch[0, self.wheel_ids].tolist()
        v_enc = spec.WHEEL_RADIUS * (jv[0] + jv[1]) / 2.0
        w_enc = spec.WHEEL_RADIUS * (jv[1] - jv[0]) / spec.WHEEL_SEP
        return jv, v_enc, w_enc, self.robot.data.root_ang_vel_b.torch[0, 2].item()

    def correct(self, v: float, w: float) -> tuple[float, float]:
        """ros2 corrector와 같은 규칙: obs 6개 → ONNX 보정량 더하고 clip · 정지 명령엔 보정 0."""
        if abs(v - self._last_cmd[0]) > 1e-3 or abs(w - self._last_cmd[1]) > 1e-3:
            self._changed_at = self._t
        self._last_cmd = (v, w)
        if self.policy is None or (v == 0.0 and w == 0.0):
            return v, w
        _, v_enc, w_enc, w_gyro = self._sense()
        obs = [[v, w, v_enc, w_enc, w_gyro, min(self._t - self._changed_at, 2.0)]]
        dv, dw = self.policy.run(None, {self.policy.get_inputs()[0].name: np.array(obs, dtype=np.float32)})[0][0]
        return (max(-spec.MAX_LIN, min(spec.MAX_LIN, v + float(dv))),
                max(-spec.MAX_ANG, min(spec.MAX_ANG, w + float(dw))))

    def send(self, v: float, w: float) -> None:
        lo, ao = self.correct(v, w)
        self.last_out = (lo, ao)
        wheels = self.wheel_model.step(lo, ao, wheel_targets, self.r, self.sep,
                                       spec.MAX_WHEEL, self.dev)
        self.robot.set_joint_velocity_target(wheels, joint_ids=self.wheel_ids)
        self.scene.write_data_to_sim()
        for _ in range(SUBSTEPS):
            self.sim.step(render=False)
        self.scene.update(POLICY_DT)
        self._t += POLICY_DT
        if self.markers is not None:
            d = self.robot.data
            yaw = self.markers.yaw_of(d.root_quat_w.torch)
            t = lambda x: torch.tensor([x], device=self.dev)
            self.markers.update(d.root_pos_w.torch, yaw, t(v), t(w),
                                d.root_lin_vel_b.torch[:, 0], d.root_ang_vel_b.torch[:, 2], self.path_w)

    def collect(self, t: float, v: float, w: float) -> list:
        jv, v_enc, w_enc, w_gyro = self._sense()
        jp = self.robot.data.joint_pos.torch[0, self.wheel_ids].tolist()
        self.oyaw += w_enc * POLICY_DT
        self.ox += v_enc * math.cos(self.oyaw) * POLICY_DT
        self.oy += v_enc * math.sin(self.oyaw) * POLICY_DT
        # sim은 단일 시계로 돌므로 msg 시각 = 기록 시각 (age 0 기준선)
        return [round(t, 3), v, w, self.last_out[0], self.last_out[1], self.ox, self.oy, self.oyaw,
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
                            help="정확한 경로 (기본: artifacts/calibration/<시각>-<label>/raw.csv)")
        parser.add_argument("--label", default=None,
                            help="CSV label · 기본 폴더 이름 (<시각>-<label>) — 없으면 sim_sensor_floor / 02-sim")
        parser.add_argument("--fit", default=None,
                            help="입력 보정: real/fit.json 또는 real/fit.json,sim/fit.json (차이) → WheelLag. "
                                 "없으면 raw plant (숨은 보정 파일은 안 읽는다)")
        parser.add_argument("--scene", default=None,
                            help="장면 클래스 (기본 DefaultScene): pkg.mod.Class 또는 path/to/file.py[:Class] — calibration.scene.Scene 서브클래스, 생성자 (condition=, actuator=)")
        parser.add_argument("--excitation", default=None,
                            help="측정 경로 클래스 (기본 DefaultCalibExcitation: 사각형→180°→역주행): pkg.mod.Class 또는 path/to/file.py[:Class] — calibration.excitation.CalibExcitation 서브클래스")
        parser.add_argument("--policy", default=None,
                            help="학습한 ONNX로 보정하며 달림 (ros2 corrector와 같은 규칙) — sim에서 정책 play · A/B")
        parser.add_argument("--watch", action="store_true", help="1회 기록 후 창을 닫을 때까지 같은 장면 반복")
        AppLauncher.add_app_launcher_args(parser)
        args = parser.parse_args(argv)
        from calibration.Calibrate import Calibrate as _Calibrate   # 표준 라이브러리만 — sim 전에 실패하게
        args.excitation_obj = _Calibrate.load_excitation(args.excitation)
        args.policy_sess = None
        if args.policy:   # sim 띄우기 전에 실패하게
            import onnxruntime as ort
            path = Path(args.policy) if os.path.isabs(args.policy) else ISAAC / args.policy
            if not path.is_file():
                parser.error(f"--policy 파일 없음: {path}")
            args.policy_sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
            dim = args.policy_sess.get_inputs()[0].shape[1]
            if dim != 6:
                parser.error(f"--policy 입력이 {dim}차원 — track 정책(6)이어야 한다")

        app_launcher = AppLauncher(args)
        simulation_app = app_launcher.app

        # AppLauncher가 pxr/physx를 초기화한 뒤에 런타임 모듈을 import한다.
        global np, torch, SimulationCfg, SimulationContext, Calibrate, DefaultScene
        global ActuatorParam, SimParam, WheelLag, wheel_targets
        import numpy as np
        import torch
        from isaaclab.sim import SimulationCfg, SimulationContext
        from calibration.Calibrate import Calibrate
        from calibration.scene.DefaultScene import DefaultScene
        from calibration.parameter.ActuatorParam import ActuatorParam
        from calibration.parameter.SimParam import SimParam
        from calibration.parameter.WheelLag import WheelLag
        from calibration.parameter.WheelLag import wheel_targets

        try:
            cls._run(args, simulation_app)
        finally:
            simulation_app.close()

    @classmethod
    def _run(cls, args, simulation_app):
        headless = not getattr(args, "visualizer", None) or "kit" not in (args.visualizer or [])
        # 보정은 사용자가 --fit으로 준 값만. 저장된 sim_calib_*.json은 읽지 않는다 (raw plant 기준).
        condition = "raw"
        excitation = args.excitation_obj
        print(f"[02-sim] excitation: {type(excitation).__module__}.{type(excitation).__name__}")
        segs = excitation.make_profile(args.speed, args.turn, args.side)
        total = excitation.total(segs)
        sim = SimulationContext(SimulationCfg(dt=DT, device=args.device))
        act = ActuatorParam()
        scene_cls = DefaultScene
        if args.scene:
            from calibration.scene.Scene import Scene
            scene_cls = Calibrate.load_class(args.scene, Scene)
        world = scene_cls(condition=condition, actuator=act)
        print(f"[02-sim] scene: {scene_cls.__module__}.{scene_cls.__name__}")
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
        tag = args.label or "sim_sensor_floor"
        out = args.out
        if out and not os.path.isabs(out):
            out = str(ISAAC / out)   # isaac/ 기준 (예: artifacts/...)
        env = "empty floor (no obstacles)" if args.room <= 0 else f"room {args.room:.1f} m"
        mode = f"FIT {', '.join(base_fit['source'])}" if base_fit else "RAW (no --fit)"
        if args.policy:
            mode += f" · POLICY {args.policy}"
        print(f"[02-sim] base 높이 {z:.3f} m · {tag} · {mode} · {env}")
        print(f"[02-sim] calib: {params.describe()}")
        markers = path_w = None
        if not headless:   # 창이 있을 때만: 명령/실제 막대 + 목표 경로(로봇 시작 자세 기준 경로를 월드로)
            from calibration.scene.CommandMarkers import CommandMarkers
            markers = CommandMarkers()
            p0 = robot.data.root_pos_w.torch[0]
            yaw0 = float(markers.yaw_of(robot.data.root_quat_w.torch)[0])
            pts = []
            line = excitation.profile_path(args.side)
            for (x0, y0), (x1, y1) in zip(line, line[1:]):
                k = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.03))
                pts += [(x0 + (x1 - x0) * i / k, y0 + (y1 - y0) * i / k) for i in range(k)]
            c, s_ = math.cos(yaw0), math.sin(yaw0)
            path_w = torch.tensor([[float(p0[0]) + c * x - s_ * y, float(p0[1]) + s_ * x + c * y,
                                    float(p0[2])] for x, y in pts], device=robot.device)
        sync = cls(sim, scene, robot, wheel_ids, wheel_model, label=tag,
                   policy=args.policy_sess, markers=markers, path_w=path_w)
        if args.label:
            sync.run_name = args.label
        speeds_txt = sync.move_speeds_text(segs)
        print(f"[02-sim] profile: side={args.side:.2f} m · {total:.1f}s · "
              f"speeds {speeds_txt} m/s · left square → 180° turn → reverse")
        cal = Calibrate(sync)
        cal.run(segs, total, out, POLICY_DT, base_fit=base_fit)

        if args.watch and not headless:
            print("[02-sim] WATCH — 창을 닫을 때까지 반복 (left square → right square)")
            sim_t = 0.0
            while simulation_app.is_running():
                v, w = excitation.cmd_at(sim_t % total, segs)
                sync.send(v, w)          # send가 물리 스텝까지 돈다 (예전엔 여기서 또 돌려 2배 빨랐음)
                sim.render()
                sim_t += POLICY_DT
                time.sleep(POLICY_DT)
        sim.stop()


if __name__ == "__main__":
    DefaultSensorSim.main()
