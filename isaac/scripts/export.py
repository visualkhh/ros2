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
parser.add_argument("--task", choices=["rl_correct", "nav2_rl"], required=True)
parser.add_argument("--checkpoint", required=True)
parser.add_argument("--out", required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)

import torch  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from tasks.agents.rsl_rl_ppo_cfg import Tb3PPORunnerCfg  # noqa: E402


def main():
    if args.task == "rl_correct":
        from tasks.rl_correct.rl_correct_env import RlCorrectEnv  # noqa: E402
        from tasks.rl_correct.rl_correct_env_cfg import RlCorrectEnvCfg  # noqa: E402

        env_cfg = RlCorrectEnvCfg()
        exp_name = os.path.splitext(args.out)[0]
    else:
        from tasks.nav2_rl.nav2_rl_env import Nav2RlEnv  # noqa: E402
        from tasks.nav2_rl.nav2_rl_env_cfg import Nav2RlEnvCfg  # noqa: E402

        env_cfg = Nav2RlEnvCfg()
        exp_name = os.path.splitext(args.out)[0]
    env_cfg.scene.num_envs = 8
    agent_cfg = Tb3PPORunnerCfg()
    agent_cfg.experiment_name = exp_name
    log_dir = "/tmp/carto_export_log"
    os.makedirs(log_dir, exist_ok=True)

    import importlib.metadata as _md
    from isaaclab_rl.rsl_rl.utils import handle_deprecated_rsl_rl_cfg  # noqa: E402

    handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
    env = RslRlVecEnvWrapper(RlCorrectEnv(env_cfg) if args.task == "rl_correct" else Nav2RlEnv(env_cfg))
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.load(args.checkpoint)

    out_dir = "/home/pank/workspaces/visualkhh/source/ros2/isaac/export"
    os.makedirs(out_dir, exist_ok=True)

    import copy  # noqa: E402

    actor = runner.alg._raw_actor
    actor.eval()

    class ActorExport(torch.nn.Module):  # noqa: E402
        def __init__(self):
            super().__init__()
            self.norm = copy.deepcopy(actor.obs_normalizer)
            self.mlp = copy.deepcopy(actor.mlp)
            self.dist = copy.deepcopy(actor.distribution)

        def forward(self, x):
            return self.dist.deterministic_output(self.mlp(self.norm(x)))

    obs_dim = actor.mlp[0].in_features
    wrapper = ActorExport().eval().to("cuda")
    torch.onnx.export(
        wrapper,
        torch.zeros(1, obs_dim, device="cuda"),
        os.path.join(out_dir, args.out),
        input_names=["obs"],
        output_names=["actions"],
        opset_version=18,
        dynamo=False,
    )
    print(f"[export] {out_dir}/{args.out} (obs_dim={obs_dim})")
    # sanity: input shape must be [1, N]
    import onnx

    model = onnx.load(f"{out_dir}/{args.out}")
    print("[export] graph inputs:", [(i.name, [d.dim_value for d in i.type.tensor_type.shape.dim]) for i in model.graph.input])
    env.close()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
