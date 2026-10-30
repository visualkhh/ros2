"""Headless smoke test: build env, step 20x with random actions, print obs/reward shapes."""
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)

import torch  # noqa: E402

from rl.wheel_drift.task.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402
from rl.wheel_drift.task.rl_correct_env import RlCorrectEnv  # noqa: E402


def main():
    # 오차·지연 0으로 두고 보정 0 → 명령대로 가는지 (물리·바퀴 부호·관측 식 확인)
    cfg = RlCorrectEnvCfg()
    cfg.scene.num_envs = 4
    cfg.tau_range = (0.05, 0.05)
    cfg.gain_range = (1.0, 1.0)
    cfg.deadband_range = (0.0, 0.0)
    cfg.cmd_delay_steps = (0, 0)
    cfg.obs_delay_steps = (0, 0)
    cfg.sep_nominal_err = 0.0
    cfg.radius_nominal_err = 0.0
    cfg.hold_range = (100.0, 100.0)
    env = RlCorrectEnv(cfg)
    obs, _ = env.reset()
    print("[smoke] obs shape:", tuple(obs["policy"].shape))
    zero = torch.zeros((4, 2), device=env.device)
    for name, v, w in (("forward 0.20", 0.20, 0.0), ("turn 1.0", 0.0, 1.0), ("diag q", 0.20, 0.40)):
        env.cmd[:] = torch.tensor([v, w], device=env.device)
        for _ in range(30):  # 1.5 s
            obs, rew, *_ = env.step(zero)
        vt, wt = env._true_twist()
        o = obs["policy"][0].tolist()
        print(f"[smoke] {name}: true v={vt.mean():+.3f} w={wt.mean():+.3f} | enc v={o[2]:+.3f} w={o[3]:+.3f} gyro w={o[4]:+.3f}")
        assert abs(vt.mean().item() - v) < 0.03 and abs(wt.mean().item() - w) < 0.12, name
    obs, rew, *_ = env.step(torch.randn((4, 2), device=env.device))
    assert torch.isfinite(obs["policy"]).all() and torch.isfinite(rew).all()
    print("[smoke] OK")
    env.close()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
