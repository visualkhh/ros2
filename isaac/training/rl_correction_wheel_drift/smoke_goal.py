"""Goal env 스모크: 오차 0에서 nominal controller가 목표 (1,0) m로 가는지."""
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)

import torch  # noqa: E402

from training.rl_correction_wheel_drift.task.goal_env import RlGoalEnv  # noqa: E402
from training.rl_correction_wheel_drift.task.goal_env_cfg import GoalEnvCfg  # noqa: E402


def main():
    cfg = GoalEnvCfg()
    cfg.scene.num_envs = 4
    cfg.tau_range = (0.05, 0.05)
    cfg.gain_range = (1.0, 1.0)
    cfg.deadband_range = (0.0, 0.0)
    cfg.cmd_delay_steps = (0, 0)
    cfg.obs_delay_steps = (0, 0)
    cfg.sep_nominal_err = 0.0
    cfg.radius_nominal_err = 0.0
    cfg.goal_radius_range = (1.0, 1.0)
    env = RlGoalEnv(cfg)
    obs, _ = env.reset()

    env.goal[:] = torch.tensor([1.0, 0.0], device=env.device)
    env.prev_dist[:] = 1.0
    env.dist[:] = 1.0
    print("[goal-smoke] obs shape:", tuple(obs["policy"].shape))
    print("[goal-smoke] start:", env._true_pos()[0].tolist(), "goal:", env.goal[0].tolist())

    zero = torch.zeros((4, 2), device=env.device)
    total = torch.zeros(4, device=env.device)
    last_p = env._true_pos().clone()
    done = False
    for i in range(240):  # 12 s
        last_p = env._true_pos().clone()
        obs, rew, term, tout, _ = env.step(zero)
        total += rew
        if i % 40 == 0:
            p = env._true_pos()
            print(f"[goal-smoke] t={i * 0.05:4.1f}s p={p[0].tolist()} "
                  f"d={env.dist[0]:.3f} rew={rew[0]:.3f}")
        if bool((term | tout).any()):
            print(f"[goal-smoke] done at t={i * 0.05:.2f}s "
                  f"term={term.tolist()} tout={tout.tolist()}")
            done = True
            break
    print("[goal-smoke] total reward:", total.tolist())
    print("[goal-smoke] last p before done:", last_p[0].tolist())
    assert done, "episode did not finish"
    assert abs(last_p[0, 0].item() - 1.0) < 0.12, "goal not reached in x"
    assert abs(last_p[0, 1].item()) < 0.12, "drifted in y"
    print("[goal-smoke] OK")
    env.close()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
