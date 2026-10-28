import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)

import torch  # noqa: E402

from tasks.nav2_rl.nav2_rl_env_cfg import Nav2RlEnvCfg  # noqa: E402
from tasks.nav2_rl.nav2_rl_env import Nav2RlEnv  # noqa: E402


def main():
    cfg = Nav2RlEnvCfg()
    cfg.scene.num_envs = 8
    env = Nav2RlEnv(cfg)
    obs, _ = env.reset()
    print("[smoke] obs keys:", list(obs.keys()), "shape:", obs["policy"].shape)
    for _ in range(20):
        act = torch.zeros((8, 2), device=env.device)
        obs, rew, term, tout, _ = env.step(act)
    print("[smoke] rew mean:", float(rew.mean()), "dists:", obs["policy"][0, 2:].tolist())
    print("[smoke] OK")
    env.close()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
