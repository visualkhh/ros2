"""rl_correct env config: velocity-tracking compensation for a 2-wheel robot."""
from __future__ import annotations

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import UsdFileCfg
from isaaclab.utils import configclass

_USDA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common", "tb3_simple.usda")


@configclass
class RlCorrectEnvCfg(DirectRLEnvCfg):
    # env
    decimation = 4  # 120 Hz sim -> 30 Hz policy
    episode_length_s = 10.0
    action_space = 2  # [dV, dW]
    observation_space = 4  # [v_cmd, w_cmd, v_odom, w_odom]
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    # robot (Waffle Pi scale: r=0.033, sep=0.287)
    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=UsdFileCfg(usd_path=_USDA),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=[".*_wheel_joint"],
                effort_limit=2.0,
                velocity_limit=8.0,
                stiffness=0.0,
                damping=40.0,  # velocity servo
            ),
        },
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=2048, env_spacing=4.0, replicate_physics=True)

    # task: command sampling + domain randomization (matches task-spec)
    max_lin = 0.22
    max_ang = 2.84
    wheel_radius = 0.033
    wheel_sep = 0.287
    radius_scale_range = (0.95, 1.05)  # per-env wheel radius error to compensate
    odom_noise_lin = 0.01
    odom_noise_ang = 0.02
    action_penalty = 0.01
    lin_err_weight = 1.0
    ang_err_weight = 0.5
