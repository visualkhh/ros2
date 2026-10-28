"""rl_correct env: learn [dV, dW] to pre-compensate tracking error.

Per episode a desired twist (v0, w0) is sampled. The policy outputs a delta,
applied twist = clip(base + delta). Sim executes with randomized wheel radius
+ 1-step latency, so the policy must learn compensation. Reward = tracking.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab import cloner
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import sample_uniform

if TYPE_CHECKING:
    from tasks.rl_correct.rl_correct_env_cfg import RlCorrectEnvCfg


class RlCorrectEnv(DirectRLEnv):
    cfg: RlCorrectEnvCfg

    def __init__(self, cfg: RlCorrectEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._left_idx, _ = self.robot.find_joints("left_wheel_joint")
        self._right_idx, _ = self.robot.find_joints("right_wheel_joint")

        self.v_cmd = torch.zeros(self.num_envs, device=self.device)
        self.w_cmd = torch.zeros(self.num_envs, device=self.device)
        self.radius_scale = torch.ones(self.num_envs, device=self.device)
        self._delayed = torch.zeros((self.num_envs, 2), device=self.device)  # 1-step latency

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        src, dest = "/World/envs/env_0", "/World/envs/env_{}"
        pos = cloner.grid_transforms(self.scene.num_envs, self.scene.cfg.env_spacing, device=self.device)[0]
        plan = cloner.clone_plan_from_env_0(src, dest, self.scene.num_envs, self.device, pos, ("/World/ground",))
        cloner.replicate(plan, stage=self.scene.stage)
        if "physx" in self.scene.physics_backend:
            self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations["robot"] = self.robot
        light_cfg = sim_utils.DistantLightCfg(intensity=2000.0, color=(1.0, 1.0, 1.0))
        light_cfg.func("/World/Light", light_cfg)

    def _sample_command(self, env_ids: torch.Tensor):
        # mostly forward, sometimes in place rotation / reverse
        mode = torch.randint(0, 3, (len(env_ids),), device=self.device)
        v = torch.zeros(len(env_ids), device=self.device)
        w = torch.zeros(len(env_ids), device=self.device)
        fwd = mode == 0
        v[fwd] = sample_uniform(0.05, self.cfg.max_lin, (int(fwd.sum()),), self.device)
        w[fwd] = sample_uniform(-0.6, 0.6, (int(fwd.sum()),), self.device)
        rot = mode == 1
        w[rot] = sample_uniform(-1.0, 1.0, (int(rot.sum()),), self.device)
        rev = mode == 2
        v[rev] = sample_uniform(-0.15, -0.03, (int(rev.sum()),), self.device)
        w[rev] = sample_uniform(-0.6, 0.6, (int(rev.sum()),), self.device)
        self.v_cmd[env_ids] = v
        self.w_cmd[env_ids] = w

    def _twist_to_wheels(self, v: torch.Tensor, w: torch.Tensor) -> torch.Tensor:
        # forward-positive wheel lin vel -> joint vel (joint + = backward)
        r = self.cfg.wheel_radius * self.radius_scale
        sep = self.cfg.wheel_sep
        wl = (v - w * sep / 2.0) / r
        wr = (v + w * sep / 2.0) / r
        return torch.stack([-wl, -wr], dim=-1)

    def _odo_from_root(self) -> tuple[torch.Tensor, torch.Tensor]:
        q = self.robot.data.root_quat_w.torch
        yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                          1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))
        fwd = torch.stack([torch.cos(yaw), torch.sin(yaw)], dim=-1)
        lin = self.robot.data.root_lin_vel_w.torch[:, :2]
        v = (lin * fwd).sum(dim=-1)
        w = self.robot.data.root_ang_vel_w.torch[:, 2]
        v = v + torch.randn_like(v) * self.cfg.odom_noise_lin
        w = w + torch.randn_like(w) * self.cfg.odom_noise_ang
        return v, w

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone()

    def _apply_action(self) -> None:
        # 1-step latency + clip to limits, then to wheel joint targets
        applied = self._delayed.clone()
        v = torch.clamp(self.v_cmd + self.actions[:, 0], -self.cfg.max_lin, self.cfg.max_lin)
        w = torch.clamp(self.w_cmd + self.actions[:, 1], -self.cfg.max_ang, self.cfg.max_ang)
        self._delayed = torch.stack([v, w], dim=-1)
        va = torch.clamp(applied[:, 0], -self.cfg.max_lin, self.cfg.max_lin)
        wa = torch.clamp(applied[:, 1], -self.cfg.max_ang, self.cfg.max_ang)
        targets = self._twist_to_wheels(va, wa)
        self.robot.set_joint_velocity_target(targets[:, 0].unsqueeze(-1), joint_ids=self._left_idx)
        self.robot.set_joint_velocity_target(targets[:, 1].unsqueeze(-1), joint_ids=self._right_idx)

    def _get_observations(self) -> dict:
        v_odom, w_odom = self._odo_from_root()
        obs = torch.stack([self.v_cmd, self.w_cmd, v_odom, w_odom], dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        v_odom, w_odom = self._odo_from_root()
        err = (self.cfg.lin_err_weight * (self.v_cmd - v_odom).abs()
               + self.cfg.ang_err_weight * (self.w_cmd - w_odom).abs())
        act = (self.actions ** 2).sum(dim=-1)
        return -err - self.cfg.action_penalty * act

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length
        return torch.zeros_like(time_out), time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        n = len(env_ids)
        root_pos = self.robot.data.default_root_state.torch[env_ids, :3].clone()
        root_pos[:, 0] += sample_uniform(-2.0, 2.0, (n,), self.device)
        root_pos[:, 1] += sample_uniform(-2.0, 2.0, (n,), self.device)
        root_pos[:, 2] = 0.075
        yaw = sample_uniform(-3.14, 3.14, (n,), self.device)
        root_quat = self.robot.data.default_root_state.torch[env_ids, 3:7].clone()
        root_quat[:, 0] = torch.cos(yaw / 2.0)
        root_quat[:, 3] = torch.sin(yaw / 2.0)
        root_quat[:, 1:3] = 0.0
        self.robot.write_root_pose_to_sim(torch.cat([root_pos, root_quat], dim=-1), env_ids)
        self.robot.write_root_velocity_to_sim(
            torch.zeros(n, 6, device=self.device), env_ids)
        ids = torch.as_tensor(env_ids, device=self.device) if not isinstance(env_ids, torch.Tensor) else env_ids
        self.robot.write_joint_state_to_sim(
            torch.zeros(n, 2, device=self.device),
            torch.zeros(n, 2, device=self.device), slice(None), ids)
        self._sample_command(ids)
        lo, hi = self.cfg.radius_scale_range
        self.radius_scale[ids] = sample_uniform(lo, hi, (len(ids),), self.device)
        self._delayed[ids] = 0.0
        self.extras.setdefault("log", {})["Metrics/track_err"] = 0.0
