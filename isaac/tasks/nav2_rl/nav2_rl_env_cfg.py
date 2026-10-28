"""nav2_rl env config: obstacle-aware velocity correction (task-spec v1)."""
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
class Nav2RlEnvCfg(DirectRLEnvCfg):
    decimation = 4  # 120 Hz sim -> 30 Hz policy
    episode_length_s = 30.0
    action_space = 2  # [dV, dW]
    observation_space = 5  # [v_nav, w_nav, d_front, d_left, d_right]
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=UsdFileCfg(usd_path=_USDA),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=[".*_wheel_joint"],
                effort_limit=2.0,
                velocity_limit=8.0,
                stiffness=0.0,
                damping=40.0,
            ),
        },
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=2048, env_spacing=10.0, replicate_physics=True)

    max_lin = 0.22
    max_ang = 2.84
    max_obs_dist = 8.0
    wheel_radius = 0.033
    wheel_sep = 0.287
    radius_scale_range = (0.95, 1.05)
    odom_noise_lin = 0.01
    odom_noise_ang = 0.02
    action_penalty = 0.01
    collision_dist = 0.16  # min sector dist -> collision
    goal_tolerance = 0.3
    n_obstacles = 5
    arena_half = 3.0  # obstacles sampled in [-3, 3]^2
    arrival_bonus = 5.0
    collision_penalty = 10.0
