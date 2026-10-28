"""Watch a trained ONNX policy drive in sim (GUI).

Usage:
  ./isaaclab.sh -p .../play.py --task rl_correct --model .../export/rl_correct_v1.onnx [--envs 4]
No --headless flag: window shows the robots. Ctrl+C to stop.
"""
import argparse
import os
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--task", choices=["rl_correct", "nav2_rl"], default="rl_correct")
parser.add_argument("--model", required=True)
parser.add_argument("--envs", type=int, default=4)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)

import numpy as np  # noqa: E402
import onnxruntime as ort  # noqa: E402
import torch  # noqa: E402


def main():
    if args.task == "rl_correct":
        from tasks.rl_correct.rl_correct_env import RlCorrectEnv  # noqa: E402
        from tasks.rl_correct.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402

        env_cfg = RlCorrectEnvCfg()
        Env = RlCorrectEnv
    else:
        from tasks.nav2_rl.nav2_rl_env import Nav2RlEnv  # noqa: E402
        from tasks.nav2_rl.nav2_rl_env_cfg import Nav2RlEnvCfg  # noqa: E402

        env_cfg = Nav2RlEnvCfg()
        Env = Nav2RlEnv
    env_cfg.scene.num_envs = args.envs
    env = Env(env_cfg)

    sess = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name
    print(f"[play] {args.model} (obs_dim={sess.get_inputs()[0].shape[1]})")

    obs, _ = env.reset()
    while True:
        o = obs["policy"].cpu().numpy().astype(np.float32)
        outs = [sess.run(None, {in_name: o[i : i + 1]})[0] for i in range(o.shape[0])]
        act = torch.as_tensor(np.concatenate(outs, axis=0), device=env.device)
        # clamp to action space (policy is delta twist)
        act[:, 0].clamp_(-0.22, 0.22)
        act[:, 1].clamp_(-2.84, 2.84)
        obs, rew, term, tout, _ = env.step(act)
        done = term | tout
        if bool(done.any()):
            print(f"[play] mean reward: {float(rew.mean()):.2f}")
            obs, _ = env.reset()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
