"""Watch a trained ONNX policy drive in sim (GUI).

Usage:
  ./isaaclab.sh -p .../play.py --model .../export/rl_correct/rl_correct_v2.onnx [--envs 4]
No --headless flag: window shows the robots. Ctrl+C to stop.
"""
import argparse
import os
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--model", required=True)
parser.add_argument("--envs", type=int, default=4)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)

import numpy as np  # noqa: E402
import onnxruntime as ort  # noqa: E402
import torch  # noqa: E402


def main():
    from rl.wheel_drift.task.rl_correct_env import RlCorrectEnv  # noqa: E402
    from rl.wheel_drift.task.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402

    env_cfg = RlCorrectEnvCfg()
    env_cfg.scene.num_envs = args.envs
    env = RlCorrectEnv(env_cfg)

    sess = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name
    print(f"[play] {args.model} (obs_dim={sess.get_inputs()[0].shape[1]})")

    obs, _ = env.reset()
    while True:
        o = obs["policy"].cpu().numpy().astype(np.float32)
        outs = [sess.run(None, {in_name: o[i : i + 1]})[0] for i in range(o.shape[0])]
        act = torch.as_tensor(np.concatenate(outs, axis=0), device=env.device)
        act = act / env.scale  # ONNX는 물리 보정량(scale 곱한 값)을 냄 → env는 [-1, 1]을 받음
        obs, rew, term, tout, _ = env.step(act)
        done = term | tout
        if bool(done.any()):
            print(f"[play] mean reward: {float(rew.mean()):.2f}")
            obs, _ = env.reset()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
