"""Train TB3 wheel-drift correction policy (track, RSL-RL PPO).

Usage (isaac/ 에서):
  ./run-training.sh [--iterations N] [--watch] [--envs N] [--no-friction] [--uneven] [--slope DEG]
  (= ./isaaclab.sh -p -m training rl_correction_wheel_drift [옵션...])
기본은 headless. `--watch` = `--visualizer kit` (창, P 키 → 스크린샷).
512 env는 느리니 `--watch --envs 16` 권장. 바퀴 마찰 랜덤(env마다)은 기본으로 켜짐, 끄려면 `--no-friction`. 바닥 = Isaac Lab 지형 (`--slope` 경사 · `--uneven` 요철).
실기기 맞춤 값(바퀴 gain · 지연 · τ)은 env_cfg.py에 박혀 있다 (측정 → 숫자 → sensor-sync-sim으로 확인).
Eval: 학습이 끝나면 같은 env·같은 seed로 정책 끔(보정 0) / 켬(policy.onnx와 같은 그래프)을 비교 → eval.json.
결과: isaac/artifacts/training/<시각>-<exp-name>/ (checkpoint·policy.onnx·eval.json·ranges.json·TensorBoard·스크린샷).
"""
import argparse
import json
import math
import sys
from pathlib import Path

ISAAC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ISAAC))

from training.trainer import Train  # noqa: E402


RANGE_KEYS = ("gain_center", "gain_range", "tau_range", "cmd_delay_steps", "obs_delay_steps", "deadband_range")
# 범위 아닌 숫자 랜덤도 ranges.json에 같이 남김
EXTRA_KEYS = ("slope_deg", "uneven", "gyro_bias")


class WheelDriftTrain(Train):
    """바퀴 드리프트 보정 정책 학습 (track, RSL-RL PPO)."""

    NAME = "rl_correct"

    @classmethod
    def add_args(cls, parser):
        parser.add_argument("--iterations", type=int, default=None,
                            help="학습 반복 횟수 (1회 = envs×64스텝 수집 + PPO 업데이트, 기본: agents cfg의 300)")
        parser.add_argument("--envs", type=int, default=512, help="학습 환경 수 (창 볼 때는 16~32 권장)")
        parser.add_argument("--friction", action=argparse.BooleanOptionalAction, default=True,
                            help="env마다 바퀴 마찰 랜덤 0.7~1.1 (Isaac Lab events, 기본 켜짐, --no-friction으로 끔)")
        parser.add_argument("--uneven", action="store_true",
                            help="지형에 요철 칸 추가 (낮은 턱 1~3 cm, Isaac Lab terrain generator)")
        parser.add_argument("--slope", type=float, default=3.0, metavar="DEG",
                            help="지형에 경사 칸 추가 (피라미드 오르막/내리막 0~DEG°, 기본 3, 0 = 없음)")
        parser.add_argument("--gyro-bias", type=float, default=None, metavar="RAD_S",
                            help="에피소드마다 자이로 영점 오차 ±RAD_S 랜덤 (기본: cfg 0.05, 0 = 끔)")
        parser.add_argument("--drive", action="store_true",
                            help="학습 대신 키보드로 직접 운전 (학습 env 그대로 · 창 · W/A/S/D · C 정책 켬/끔 · R 리셋 · F 카메라)")
        parser.add_argument("--policy", default=None, metavar="ONNX",
                            help="--drive에서 보정에 쓸 정책 (C 키로 켬/끔, 기본 켬)")
        parser.add_argument("--speed", type=float, default=0.15, help="--drive 전진 [m/s] (Shift = 0.3배)")
        parser.add_argument("--turn", type=float, default=1.0, help="--drive 회전 [rad/s]")
    @classmethod
    def before_launch(cls, args, parser):
        if args.policy and not Path(args.policy).is_file():
            parser.error(f"--policy 파일 없음: {args.policy}")
        if args.drive and "kit" not in (args.visualizer or []):
            parser.error("--drive는 창이 필요 — --watch를 같이")
    def run(self):
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
        from rsl_rl.runners import OnPolicyRunner  # noqa: E402

        from training.rl_correction_wheel_drift.agent import RlCorrectAgentCfg  # noqa: E402
        from training.rl_correction_wheel_drift.env import RlCorrectEnv  # noqa: E402
        from training.rl_correction_wheel_drift.env_cfg import RlCorrectEnvCfg  # noqa: E402

        args, log_dir = self.args, str(self.run_dir)
        env_cfg = RlCorrectEnvCfg()
        env_cfg.scene.num_envs = 1 if args.drive else args.envs
        from training.rl_correction_wheel_drift.env_cfg import EventCfg, make_terrain  # noqa: E402

        env_cfg.events = EventCfg() if args.friction else None
        env_cfg.slope_deg, env_cfg.uneven = args.slope, args.uneven
        env_cfg.terrain = make_terrain(args.slope, args.uneven)
        if args.gyro_bias is not None:
            env_cfg.gyro_bias = args.gyro_bias
        self.add_snap_visualizer(env_cfg)
        # 창·스냅샷이 있을 때만 명령/실제 막대 + 목표 경로 표시 (headless 학습엔 영향 없음)
        env_cfg.debug_vis = bool(self.watch or getattr(args, "snap", None))
        # 이번 학습의 랜덤 범위 기록 (실기기 맞춤 값은 env_cfg.py에 박혀 있다)
        ranges = {k: list(getattr(env_cfg, k)) for k in RANGE_KEYS} | {k: getattr(env_cfg, k) for k in EXTRA_KEYS}
        print(f"[train] ranges {ranges}")
        (self.run_dir / "ranges.json").write_text(json.dumps(ranges, indent=2, ensure_ascii=False) + "\n")

        if args.drive:
            return self.drive(RlCorrectEnv, env_cfg)

        agent_cfg = RlCorrectAgentCfg()
        agent_cfg.experiment_name = args.exp_name
        agent_cfg.log_dir = log_dir
        if args.iterations:
            agent_cfg.max_iterations = args.iterations

        env = RslRlVecEnvWrapper(RlCorrectEnv(env_cfg))
        self.env = env
        self.install_snap(env.unwrapped)
        import importlib.metadata as _md
        from isaaclab_rl.rsl_rl.utils import handle_deprecated_rsl_rl_cfg  # noqa: E402

        from agents.base_agent import check  # noqa: E402

        check(agent_cfg)   # agent.py가 안 채운 값이 있으면 이름과 함께 멈춤
        handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

        from utils.onnx_export import export_actor  # noqa: E402

        path = self.run_dir / "policy.onnx"   # RL 관례 이름 (폴더 이름에 exp-name이 있다)
        export_actor(runner.alg._raw_actor, env_cfg.action_scale, str(path))
        print(f"[train] exported {path}")
        import torch
        with torch.inference_mode():   # rsl_rl이 rollout을 inference_mode로 돌려 env 버퍼가 inference 텐서 → 밖에서 reset하면 RuntimeError
            result = self.evaluate(env.unwrapped, runner.alg._raw_actor, env_cfg.action_scale, path)
        (self.run_dir / "eval.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        env.close()

    def drive(self, env_cls, env_cfg) -> None:
        """학습 env(지형 경사·요철 · 바퀴 마찰/gain/τ 랜덤 그대로, 로봇 1대)를 키보드로 운전. 랜덤 명령 대신 키 입력이 명령.

        정책은 ros2 corrector와 같은 규칙: 관측 6개 → ONNX 보정량(물리값) → env가 [-1,1]로 받아 scale 곱함 · 정지 명령엔 0.
        """
        import time

        import omni.kit.app
        import torch

        args = self.args
        env_cfg.episode_length_s = 1e6   # 시간 초과 리셋 없음 (R 키로만)
        raw = env_cls(env_cfg)
        self.env = raw
        sess = None
        if args.policy:
            import onnxruntime as ort
            sess = ort.InferenceSession(args.policy, providers=["CPUExecutionProvider"])
        use_policy = sess is not None
        from commands import keyboard as kb   # 키 상태 폴링 (공용)

        obs, _ = raw.reset()
        raw.look_at(raw.robot.data.root_pos_w.torch[0].tolist())
        last_cmd, prev_keys, t_print = None, set(), 0.0
        print("[drive] W/A/S/D(↑←↓→) 주행 · Shift 느리게 · X 정지 · C 정책 켬/끔 · R 리셋(제자리 · 바퀴 gain/τ 새로) · "
              "F 카메라를 로봇 쪽으로 · 마우스 휠 줌 · 창 닫으면 끝", flush=True)
        print(f"[drive] 정책 {'켬 ' + args.policy if use_policy else '없음 (바이패스)'}", flush=True)
        app = omni.kit.app.get_app()
        while app.is_running():
            t0 = time.monotonic()
            keys = kb.pressed()
            new = keys - prev_keys
            prev_keys = keys
            if "C" in new and sess is not None:
                use_policy = not use_policy
                print(f"[drive] 정책 {'켬' if use_policy else '끔 (바이패스)'}", flush=True)
            if "R" in new:
                obs, _ = raw.reset()
            if "F" in new:   # 카메라는 평소 자유 (따라가게 고정하면 마우스 줌·회전이 덮어씌워짐)
                raw.look_at(raw.robot.data.root_pos_w.torch[0].tolist())
            v, w = kb.drive_cmd(keys, args.speed, args.turn)
            cmd = (v, w)
            raw.cmd[:] = torch.tensor(cmd, device=raw.device)
            raw.hold[:] = 1e9            # env가 랜덤 명령으로 바꾸지 않게
            raw.hold_total[:] = 2.0      # 노란 점 = 지금 명령대로 2 s 갈 길
            if cmd != last_cmd:
                raw.t_change[:] = 0.0    # 관측 t_change (학습 때와 같은 뜻)
                last_cmd = cmd
            act = torch.zeros((1, 2), device=raw.device)
            if use_policy:
                d = sess.run(None, {"obs": obs["policy"][:1].detach().cpu().numpy().astype("float32")})[0]
                act = (torch.as_tensor(d, device=raw.device) / raw.scale).clamp(-1.0, 1.0)
            obs, *_ = raw.step(act)
            if t0 - t_print > 0.5 and (v or w):   # 움직일 때만
                t_print = t0
                vt, wt = raw.true_twist()
                dl = raw.delta[0].tolist()
                print(f"[drive] 명령 v {v:+.2f} w {w:+.2f} | 실제 v {float(vt[0]):+.3f} w {float(wt[0]):+.3f} | "
                      f"보정 Δv {dl[0]:+.3f} Δw {dl[1]:+.3f} | 정책 {'켬' if use_policy else '끔'}", flush=True)
            time.sleep(max(0.0, raw.policy_dt - (time.monotonic() - t0)))   # 실시간 (정책 주기 50 ms)
        raw.close()

    def evaluate(self, raw, actor, action_scale, onnx_path, episodes: int = 3, seed: int = 7) -> dict:
        """sim Before/After — 같은 env·같은 seed(같은 랜덤화·명령)로 정책 끔(보정 0) / 켬.

        정책은 policy.onnx와 같은 그래프(utils.onnx_export._Actor)로 돌린다. 지표는 실제 몸체 속도 vs 명령
        RMS (보상과 같은 기준, 정지 명령 제외). onnxruntime이 있으면 ONNX 출력이 같은지도 확인한다.
        """
        import random

        import torch

        from utils.onnx_export import _Actor  # noqa: E402

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
                    v, w = raw.true_twist()
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
