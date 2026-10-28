"""Headless smoke test: build env, step 20x with random actions, print obs/reward shapes."""
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)

import torch  # noqa: E402

from tasks.rl_correct.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402
from tasks.rl_correct.rl_correct_env import RlCorrectEnv  # noqa: E402


def main():
    cfg = RlCorrectEnvCfg()
    cfg.scene.num_envs = 8
    env = RlCorrectEnv(cfg)
    obs, _ = env.reset()
    print("[smoke] obs keys:", list(obs.keys()), "shape:", obs["policy"].shape)
    for _ in range(20):
        act = torch.zeros((8, 2), device=env.device)
        obs, rew, term, tout, _ = env.step(act)
    print("[smoke] rew mean:", float(rew.mean()), "obs sample:", obs["policy"][0].tolist())
    print("[smoke] OK")
    env.close()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
