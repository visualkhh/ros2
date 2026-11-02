"""Train TB3 wheel-drift correction policy (track, RSL-RL PPO).

Usage (isaac/ 에서):
  ./run-training.sh [--iterations N] [--watch] [--envs N] [--no-friction]
      [--fit artifacts/calibration/<real>/fit.json]
  (= ./isaaclab.sh -p -m training rl_correction_wheel_drift [옵션...])
기본은 headless. `--watch` = `--visualizer kit` (창 + TRAINING FIT 보정값 창, 읽기만, P 키 → 스크린샷).
512 env는 느리니 `--watch --envs 16` 권장. 마찰 패치(파랑/주황)는 기본으로 켜짐, 끄려면 `--no-friction`.
Fit: `--fit real/fit.json,sim/fit.json` → real − sim 차이를 바퀴 랜덤 범위의 중심으로 (폭은 그대로).
     학습 env와 calibration sim은 같은 바퀴 모터(ActuatorParam)라 sim fit = 학습 env의 raw 응답.
     sim fit은 --fit 없이(raw), 지금 ActuatorParam과 같은 damping으로 잰 것이어야 한다 (fit.json의 plant로 확인).
     1개만 주면 real 값 그대로 (학습 env plant를 이상적이라고 보는 것 — 지연이 이중으로 들어감).
Latency: `--latency before.csv` = rl-only Before 기록(경로 rl, enable:=false) → RL 경로(Pi→PC→Pi) 지연을
     명령 지연 범위에 더하고 관측 지연 범위의 중심으로 (폭은 그대로). fit의 지연은 경로 direct로 잰 값이라 따로 더한다.
Eval: 학습이 끝나면 같은 env·같은 seed로 정책 끔(보정 0) / 켬(policy.onnx와 같은 그래프)을 비교 → eval.json.
결과: isaac/artifacts/training/<시각>-<exp-name>/ (checkpoint·policy.onnx·eval.json·TensorBoard·스크린샷,
     `--fit`/`--latency`면 base-fit.json도).
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

ISAAC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ISAAC))

from training.Train import Train  # noqa: E402


def apply_fit(env_cfg, wheel) -> dict:
    """real 바퀴 fit(WheelParam) → env cfg 랜덤 범위. 폭은 기존 그대로, 중심만 real로.

    gain은 기존 gain_range(1.0 중심) 샘플에 좌/우 real gain을 곱한다(`gain_center`).
    deadband는 fit이 0이면 기존 범위 유지 — 사각형 excitation엔 저속 구간이 없어 못 잰 값이다.
    """
    def around(rng, center, lo=0.0):
        half = (rng[1] - rng[0]) / 2.0
        return (round(max(lo, center - half), 4), round(center + half, 4))

    env_cfg.gain_center = wheel.gains()
    env_cfg.tau_range = around(env_cfg.tau_range, wheel.tau_s, lo=0.01)
    half = (env_cfg.cmd_delay_steps[1] - env_cfg.cmd_delay_steps[0]) // 2
    env_cfg.cmd_delay_steps = (max(0, wheel.delay_steps - half), wheel.delay_steps + half)
    if wheel.deadband > 0:
        env_cfg.deadband_range = around(env_cfg.deadband_range, wheel.deadband)


def rl_latency(path) -> dict:
    """rl-only Before CSV(eval_recorder, 경로 rl) → RL 경로 지연.

    관측 나이 = 입력 수신 시각 − odom·imu stamp (정책이 볼 때 센서가 얼마나 묵었나) 중앙값.
    corrector 처리 = 응답 수신 − 입력 수신 중앙값. 명령 추가 지연 ≈ 2 × 관측 나이 + 처리
    (Pi→PC 한 번 + PC→Pi 한 번).
    """
    # ponytail: 관측 나이에는 센서 주기 절반이 섞여 있어 왕복을 조금 크게 잡는다 — 정확히 하려면 Pi 쪽 stamp로 왕복을 직접 잰다
    def num(r, k):
        try:
            return float(r[k])
        except (KeyError, TypeError, ValueError):
            return math.nan

    def med(xs):
        xs = sorted(x for x in xs if x == x)
        return xs[len(xs) // 2] if xs else math.nan

    rows = list(csv.DictReader(open(path)))
    age = med([num(r, "t_ros") - num(r, k) for r in rows for k in ("t_odom", "t_imu")])
    proc = med([num(r, "t_out") - num(r, "t_ros") for r in rows])
    if not (0.0 <= age < 1.0) or not (0.0 <= proc < 1.0):
        raise ValueError(f"지연이 이상함 (관측 나이 {age}, 처리 {proc} s) — PC·Pi 시계 동기(chrony) 확인, "
                         f"경로 rl · rl-only로 기록한 CSV인지 확인")
    dt = 0.05   # 정책 주기 (robot_spec.CONTROL_HZ = 20)
    return {"source": str(Path(path).resolve()), "obs_age_ms": round(age * 1000, 1),
            "proc_ms": round(proc * 1000, 1), "obs_delay_steps": int(round(age / dt)),
            "cmd_extra_steps": int(round((2 * age + proc) / dt))}


def apply_latency(env_cfg, lat: dict) -> None:
    """RL 경로 지연 → 명령 지연 범위는 그만큼 밀고, 관측 지연 범위는 그 값을 중심으로 (폭 유지)."""
    lo, hi = env_cfg.cmd_delay_steps
    env_cfg.cmd_delay_steps = (lo + lat["cmd_extra_steps"], hi + lat["cmd_extra_steps"])
    half = (env_cfg.obs_delay_steps[1] - env_cfg.obs_delay_steps[0]) // 2
    s = lat["obs_delay_steps"]
    env_cfg.obs_delay_steps = (max(0, s - half), s + half)


RANGE_KEYS = ("gain_center", "gain_range", "tau_range", "cmd_delay_steps", "obs_delay_steps", "deadband_range")


class WheelDriftTrain(Train):
    """바퀴 드리프트 보정 정책 학습 (track, RSL-RL PPO)."""

    NAME = "rl_correct"

    @classmethod
    def add_args(cls, parser):
        parser.add_argument("--iterations", type=int, default=None,
                            help="학습 반복 횟수 (1회 = envs×64스텝 수집 + PPO 업데이트, 기본: agents cfg의 300)")
        parser.add_argument("--envs", type=int, default=512, help="학습 환경 수 (창 볼 때는 16~32 권장)")
        parser.add_argument("--friction", action=argparse.BooleanOptionalAction, default=True,
                            help="env별 바닥 마찰 랜덤 + 미끄럼/끈적 패치 (기본 켜짐, --no-friction으로 끔)")
        parser.add_argument("--patch-count", type=int, default=4,
                            help="env 안 국소 마찰 패치 수 (0이면 base friction만)")
        parser.add_argument("--uneven", action="store_true",
                            help="env 안 요철(울퉁불퉁) 지형 켜기 (낮은 턱, 기하학적 외란)")
        parser.add_argument("--uneven-count", type=int, default=4,
                            help="env 안 요철 패치 수")
        parser.add_argument("--fit", default=None,
                            help="real/fit.json,sim/fit.json → 차이를 바퀴 랜덤 범위 중심으로 (1개면 real 값 그대로)")
        parser.add_argument("--latency", default=None,
                            help="rl-only Before CSV (경로 rl, enable:=false) → RL 경로 지연을 명령·관측 지연 범위에")

    @classmethod
    def before_launch(cls, args, parser):
        # sim 띄우기 전에 fit부터 읽어서 잘못된 경로면 바로 실패
        args.base_fit = None
        if args.fit:
            from calibration.Calibrate import Calibrate
            from calibration.parameter.ActuatorParam import ActuatorParam
            paths = [p for p in args.fit.split(",") if p.strip()]
            if len(paths) == 2:   # sim fit이 지금 학습 env와 같은 모터 설정으로 잰 것인지
                plant = json.loads(Path(paths[1]).expanduser().read_text()).get("plant")
                want = ActuatorParam().damping
                if not plant or abs(float(plant.get("actuator_damping", -1)) - want) > 1e-9:
                    parser.error(f"sim fit의 damping({plant and plant.get('actuator_damping')})이 학습 env의 "
                                 f"ActuatorParam.damping({want})과 다르다 — 같은 값으로 sensor-sync-sim을 다시 재라")
            args.base_fit = Calibrate.load_fit(args.fit)
        args.latency_info = None
        if args.latency:
            try:
                args.latency_info = rl_latency(args.latency)
            except (OSError, ValueError) as exc:
                parser.error(f"--latency: {exc}")

    def run(self):
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
        from rsl_rl.runners import OnPolicyRunner  # noqa: E402

        from calibration.parameter.SimParam import SimParam  # noqa: E402
        from training.rl_correction_wheel_drift.agents.rsl_rl_ppo_cfg import Tb3PPORunnerCfg  # noqa: E402
        from training.rl_correction_wheel_drift.task.rl_correct_env import RlCorrectEnv  # noqa: E402
        from training.rl_correction_wheel_drift.task.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402

        args, base_fit, lat, log_dir = self.args, self.args.base_fit, self.args.latency_info, str(self.run_dir)
        env_cfg = RlCorrectEnvCfg()
        env_cfg.scene.num_envs = args.envs
        env_cfg.randomize_friction = args.friction
        env_cfg.friction_patch_count = args.patch_count
        if args.uneven:
            env_cfg.randomize_uneven = True
            env_cfg.uneven_patch_count = args.uneven_count
        if base_fit is not None:
            apply_fit(env_cfg, SimParam.from_dict(base_fit).wheel)
        if lat is not None:
            apply_latency(env_cfg, lat)
        if base_fit is not None or lat is not None:
            ranges = {k: list(getattr(env_cfg, k)) for k in RANGE_KEYS}
            print(f"[train] fit={base_fit and base_fit['source']} latency={lat} → {ranges}")
            (self.run_dir / "base-fit.json").write_text(json.dumps(
                {"source": base_fit and base_fit["source"],
                 "fit": base_fit and {k: v for k, v in base_fit.items() if k != "source"},
                 "latency": lat, "ranges": ranges}, indent=2, ensure_ascii=False) + "\n")

        agent_cfg = Tb3PPORunnerCfg()
        agent_cfg.experiment_name = args.exp_name
        agent_cfg.logger = "tensorboard"
        agent_cfg.log_dir = log_dir
        if args.iterations:
            agent_cfg.max_iterations = args.iterations

        env = RslRlVecEnvWrapper(RlCorrectEnv(env_cfg))
        if self.watch:
            from training.rl_correction_wheel_drift.FitPanel import FitPanel  # noqa: E402
            self.panel = FitPanel(env_cfg, base_fit["source"][0] if base_fit else None, lat)  # 참조 유지
        import importlib.metadata as _md
        from isaaclab_rl.rsl_rl.utils import handle_deprecated_rsl_rl_cfg  # noqa: E402

        handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

        from training.onnx.onnx_export import export_actor  # noqa: E402

        path = self.run_dir / "policy.onnx"   # RL 관례 이름 (폴더 이름에 exp-name이 있다)
        export_actor(runner.alg._raw_actor, env_cfg.action_scale, str(path))
        print(f"[train] exported {path}")
        import torch
        with torch.inference_mode():   # rsl_rl이 rollout을 inference_mode로 돌려 env 버퍼가 inference 텐서 → 밖에서 reset하면 RuntimeError
            result = self.evaluate(env.unwrapped, runner.alg._raw_actor, env_cfg.action_scale, path)
        (self.run_dir / "eval.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        env.close()

    def evaluate(self, raw, actor, action_scale, onnx_path, episodes: int = 3, seed: int = 7) -> dict:
        """sim Before/After — 같은 env·같은 seed(같은 랜덤화·명령)로 정책 끔(보정 0) / 켬.

        정책은 policy.onnx와 같은 그래프(onnx_export._Actor)로 돌린다. 지표는 실제 몸체 속도 vs 명령
        RMS (보상과 같은 기준, 정지 명령 제외). onnxruntime이 있으면 ONNX 출력이 같은지도 확인한다.
        """
        import random

        import torch

        from training.onnx.onnx_export import _Actor  # noqa: E402

        policy = _Actor(actor, action_scale).eval().to(raw.device)
        n, steps = raw.num_envs, int(round(raw.cfg.episode_length_s / raw.policy_dt))
        zero = torch.zeros((n, 2), device=raw.device)

        def phase(use_policy: bool) -> dict:
            torch.manual_seed(seed)
            random.seed(seed)
            sv = sw = cnt = 0.0
            for _ in range(episodes):
                obs, _ = raw.reset()
                for _ in range(steps):
                    with torch.no_grad():
                        act = (policy(obs["policy"]) / raw.scale).clamp(-1.0, 1.0) if use_policy else zero
                    obs, *_ = raw.step(act)
                    v, w = raw._true_twist()
                    moving = raw.cmd.abs().sum(dim=-1) > 0
                    sv += float(((raw.cmd[:, 0] - v) ** 2)[moving].sum())
                    sw += float(((raw.cmd[:, 1] - w) ** 2)[moving].sum())
                    cnt += float(moving.sum())
            return {"rms_v": math.sqrt(sv / max(cnt, 1.0)), "rms_w": math.sqrt(sw / max(cnt, 1.0))}

        before, after = phase(False), phase(True)
        result = {"episodes": episodes, "envs": n, "seed": seed, "before": before, "after": after,
                  "improve_pct": {k: (before[k] - after[k]) / max(before[k], 1e-9) * 100.0 for k in before}}
        try:   # 배포 파일(ONNX)과 평가 그래프가 같은지
            import onnxruntime as ort
            obs, _ = raw.reset()
            import numpy as np
            o = obs["policy"][:64]
            sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
            rows = o.detach().cpu().numpy().astype("float32")   # ONNX 입력은 [1, 6] 고정 (ROS corrector와 같게) → 한 줄씩
            onnx_out = np.concatenate([sess.run(None, {"obs": r[None]})[0] for r in rows])
            with torch.no_grad():
                torch_out = policy(o).cpu().numpy()
            result["onnx_max_abs_diff"] = float(abs(onnx_out - torch_out).max())
        except ImportError:
            result["onnx_max_abs_diff"] = None   # onnxruntime 없음 — 그래프 비교 생략
        for k in before:
            print(f"[eval] {k}: 끔 {before[k]:.4f} → 켬 {after[k]:.4f} ({result['improve_pct'][k]:+.1f}%)")
        print(f"[eval] onnx 차이: {result['onnx_max_abs_diff']}")
        return result


def main(argv=None):
    return WheelDriftTrain.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
