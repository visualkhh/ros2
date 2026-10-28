"""Train TB3 correction policies (RSL-RL PPO).

Usage (Lab env):
  cd /home/pank/pa-course/IsaacLab-6.1
  ./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/train.py --task rl_correct
  ./isaaclab.sh -p .../train.py --task nav2_rl --headless
Watch: run WITHOUT --headless (viewport shows robots). Logs: isaac/logs/<task>/.
"""
import argparse
import os
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--task", choices=["rl_correct", "nav2_rl"], required=True)
parser.add_argument("--max-iterations", type=int, default=None)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)

import torch  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from tasks.agents.rsl_rl_ppo_cfg import Tb3PPORunnerCfg  # noqa: E402

if args.task == "rl_correct":
    from tasks.rl_correct.rl_correct_env import RlCorrectEnv  # noqa: E402
    from tasks.rl_correct.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402

    env_cfg = RlCorrectEnvCfg()
    env_cfg.scene.num_envs = 512
    exp_name = "rl_correct"
else:
    from tasks.nav2_rl.nav2_rl_env import Nav2RlEnv  # noqa: E402
    from tasks.nav2_rl.nav2_rl_env_cfg import Nav2RlEnvCfg  # noqa: E402

    env_cfg = Nav2RlEnvCfg()
    env_cfg.scene.num_envs = 512
    exp_name = "nav2_rl"


def main():
    log_dir = f"/home/pank/workspaces/visualkhh/source/ros2/isaac/logs/{exp_name}"
    os.makedirs(log_dir, exist_ok=True)
    agent_cfg = Tb3PPORunnerCfg()
    agent_cfg.experiment_name = exp_name
    agent_cfg.logger = "tensorboard"
    agent_cfg.log_dir = log_dir
    if args.max_iterations:
        agent_cfg.max_iterations = args.max_iterations

    env = RslRlVecEnvWrapper(RlCorrectEnv(env_cfg) if args.task == "rl_correct" else Nav2RlEnv(env_cfg))
    import importlib.metadata as _md
    from isaaclab_rl.rsl_rl.utils import handle_deprecated_rsl_rl_cfg  # noqa: E402

    handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
    agent_dict = agent_cfg.to_dict()
    runner = OnPolicyRunner(env, agent_dict, log_dir=log_dir, device=agent_cfg.device)
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    from isaaclab_rl.rsl_rl.exporter import export_policy_as_onnx  # noqa: E402

    export_policy_as_onnx(runner.alg.actor_critic, normalizer=None, path=log_dir, filename=f"{exp_name}.onnx")
    print(f"[train] exported {log_dir}/{exp_name}.onnx")
    env.close()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
