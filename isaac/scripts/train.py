"""Train TB3 correction policies (RSL-RL PPO).

Usage (Lab env):
  cd /home/pank/pa-course/IsaacLab-6.1
  ./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/train.py --task rl_correct
Watch: run WITHOUT --headless (viewport shows robots). Logs: isaac/logs/<task>/.
"""
import argparse
import os
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--task", choices=["rl_correct"], default="rl_correct")
parser.add_argument("--max-iterations", type=int, default=None)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)


from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from tasks.agents.rsl_rl_ppo_cfg import Tb3PPORunnerCfg  # noqa: E402

from tasks.rl_correct.rl_correct_env import RlCorrectEnv  # noqa: E402
from tasks.rl_correct.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402

env_cfg = RlCorrectEnvCfg()
env_cfg.scene.num_envs = 512
exp_name = "rl_correct"


def main():
    from datetime import datetime  # noqa: E402

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = f"/home/pank/workspaces/visualkhh/source/ros2/isaac/logs/{exp_name}/{stamp}"
    os.makedirs(log_dir, exist_ok=True)
    agent_cfg = Tb3PPORunnerCfg()
    agent_cfg.experiment_name = exp_name
    agent_cfg.logger = "tensorboard"
    agent_cfg.log_dir = log_dir
    if args.max_iterations:
        agent_cfg.max_iterations = args.max_iterations

    env = RslRlVecEnvWrapper(RlCorrectEnv(env_cfg))
    import importlib.metadata as _md
    from isaaclab_rl.rsl_rl.utils import handle_deprecated_rsl_rl_cfg  # noqa: E402

    handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
    agent_dict = agent_cfg.to_dict()
    runner = OnPolicyRunner(env, agent_dict, log_dir=log_dir, device=agent_cfg.device)
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    from tasks.common.onnx_export import export_actor  # noqa: E402

    path = os.path.join(log_dir, f"{exp_name}.onnx")
    export_actor(runner.alg._raw_actor, env_cfg.action_scale, path)
    print(f"[train] exported {path}")
    env.close()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
