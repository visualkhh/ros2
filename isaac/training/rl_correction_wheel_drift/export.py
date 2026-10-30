"""Export a trained checkpoint to ONNX (matches ROS node input [1, N] float32).

Usage:
  ./isaaclab.sh -p .../export.py --checkpoint /path/model_2.pt --out name.onnx
Output: isaac/export/<name.onnx>
"""
import argparse
import os
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", required=True)
parser.add_argument("--out", required=True)
parser.add_argument("--task", choices=("track", "goal"), default="track")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)


from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from training.rl_correction_wheel_drift.agents.rsl_rl_ppo_cfg import Tb3PPORunnerCfg  # noqa: E402


def main():
    if args.task == "goal":
        from training.rl_correction_wheel_drift.task.goal_env import RlGoalEnv as TaskEnv  # noqa: E402
        from training.rl_correction_wheel_drift.task.goal_env_cfg import GoalEnvCfg as TaskCfg  # noqa: E402
        out_sub = "rl_goal"
    else:
        from training.rl_correction_wheel_drift.task.rl_correct_env import RlCorrectEnv as TaskEnv  # noqa: E402
        from training.rl_correction_wheel_drift.task.rl_correct_env_cfg import RlCorrectEnvCfg as TaskCfg  # noqa: E402
        out_sub = "rl_correct"

    env_cfg = TaskCfg()
    exp_name = os.path.splitext(args.out)[0]
    env_cfg.scene.num_envs = 8
    agent_cfg = Tb3PPORunnerCfg()
    agent_cfg.experiment_name = exp_name
    log_dir = "/tmp/rl_wheel_drift_export_log"
    os.makedirs(log_dir, exist_ok=True)

    import importlib.metadata as _md
    from isaaclab_rl.rsl_rl.utils import handle_deprecated_rsl_rl_cfg  # noqa: E402

    handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
    env = RslRlVecEnvWrapper(TaskEnv(env_cfg))
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.load(args.checkpoint)

    out_dir = f"/home/pank/workspaces/visualkhh/source/ros2/isaac/export/{out_sub}"
    os.makedirs(out_dir, exist_ok=True)

    from tasks.common.onnx_export import export_actor  # noqa: E402

    obs_dim = export_actor(runner.alg._raw_actor, env_cfg.action_scale, os.path.join(out_dir, args.out))
    print(f"[export] {out_dir}/{args.out} (obs_dim={obs_dim})")
    # sanity: input shape must be [1, N]
    import onnx

    model = onnx.load(f"{out_dir}/{args.out}")
    print("[export] graph inputs:", [(i.name, [d.dim_value for d in i.type.tensor_type.shape.dim]) for i in model.graph.input])
    env.close()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
