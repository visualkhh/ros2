"""Export a trained checkpoint to ONNX (matches ROS node input [1, N] float32).

Usage:
  ./isaaclab.sh -p .../export.py --task rl_correct --checkpoint /path/model_2.pt --out name.onnx
Output: isaac/export/<name.onnx>
"""
import argparse
import os
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--task", choices=["rl_correct"], default="rl_correct")
parser.add_argument("--checkpoint", required=True)
parser.add_argument("--out", required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)


from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from tasks.agents.rsl_rl_ppo_cfg import Tb3PPORunnerCfg  # noqa: E402


def main():
    from tasks.rl_correct.rl_correct_env import RlCorrectEnv  # noqa: E402
    from tasks.rl_correct.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402

    env_cfg = RlCorrectEnvCfg()
    exp_name = os.path.splitext(args.out)[0]
    env_cfg.scene.num_envs = 8
    agent_cfg = Tb3PPORunnerCfg()
    agent_cfg.experiment_name = exp_name
    log_dir = "/tmp/carto_export_log"
    os.makedirs(log_dir, exist_ok=True)

    import importlib.metadata as _md
    from isaaclab_rl.rsl_rl.utils import handle_deprecated_rsl_rl_cfg  # noqa: E402

    handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
    env = RslRlVecEnvWrapper(RlCorrectEnv(env_cfg))
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.load(args.checkpoint)

    out_dir = f"/home/pank/workspaces/visualkhh/source/ros2/isaac/export/{args.task}"
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
