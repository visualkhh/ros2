"""nav2_rl env: learn [dV, dW] to shape Nav2-like commands around obstacles.

Base command = pure-pursuit to a random goal (mimics Nav2 output).
Policy sees [v_nav, w_nav, d_front, d_left, d_right] (analytic sector dists).
Reward = progress + arrival - collision. Placed boxes are static colliders.
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
    from tasks.nav2_rl.nav2_rl_env_cfg import Nav2RlEnvCfg


def _spawn_box(stage, path: str, cx: float, cy: float, hx: float, hy: float, hz: float = 0.5):
    """Static cuboid collider (pxr USD)."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    prim = stage.DefinePrim(Sdf.Path(path), "Xform")
    UsdGeom.Xform(prim).AddTranslateOp().Set(Gf.Vec3d(cx, cy, hz / 2.0))
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path + "/geom"))
    cube.GetSizeAttr().Set(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(hx * 2.0, hy * 2.0, hz))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return prim


class Nav2RlEnv(DirectRLEnv):
    cfg: Nav2RlEnvCfg

    def __init__(self, cfg: Nav2RlEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._left_idx, _ = self.robot.find_joints("wheel_left_joint")
        self._right_idx, _ = self.robot.find_joints("wheel_right_joint")
        self._arm_idx, _ = self.robot.find_joints(
            ["joint1", "joint2", "joint3", "joint4", "joint5",
             "gripper_left_joint", "gripper_right_joint"])

    # ------------------------------------------------------------- scene
    def _setup_scene(self):
        self.v_nav = torch.zeros(self.num_envs, device=self.device)
        self.w_nav = torch.zeros(self.num_envs, device=self.device)
        self.goal = torch.zeros((self.num_envs, 2), device=self.device)
        self.prev_goal_dist = torch.zeros(self.num_envs, device=self.device)
        self.radius_scale = torch.ones(self.num_envs, device=self.device)
        self._delayed = torch.zeros((self.num_envs, 2), device=self.device)
        # per-env static boxes: (cx, cy, hx, hy)
        self.boxes = torch.zeros((self.num_envs, self.cfg.n_obstacles, 4), device=self.device)
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

        # fixed per-env obstacle layout (seeded) — static colliders
        g = torch.Generator(device="cpu")
        g.manual_seed(7)
        H = self.cfg.arena_half
        for i in range(self.scene.num_envs):
            for k in range(self.cfg.n_obstacles):
                for _ in range(20):  # keep center clear
                    cx = (torch.rand(1, generator=g).item() * 2 - 1) * H
                    cy = (torch.rand(1, generator=g).item() * 2 - 1) * H
                    if cx * cx + cy * cy > 1.5 * 1.5:
                        break
                hx = 0.2 + torch.rand(1, generator=g).item() * 0.4
                hy = 0.2 + torch.rand(1, generator=g).item() * 0.4
                self.boxes[i, k] = torch.tensor([cx, cy, hx, hy])
                ox, oy = pos[i][0].item(), pos[i][1].item()
                _spawn_box(self.scene.stage, f"/World/envs/env_{i}/obstacles/box_{k}",
                           ox + cx, oy + cy, hx, hy)

    # ------------------------------------------------------------- helpers
    def _sector_dists(self) -> torch.Tensor:
        """Analytic min ray distance per sector (front/left/right) vs boxes."""
        p = self.robot.data.root_pos_w.torch[:, :2]  # world; boxes stored in env-local -> shift by origin
        q0 = self.robot.data.root_quat_w.torch
        yaw = torch.atan2(2.0 * (q0[:, 0] * q0[:, 3] + q0[:, 1] * q0[:, 2]),
                          1.0 - 2.0 * (q0[:, 2] ** 2 + q0[:, 3] ** 2))
        rel = p - self.scene.env_origins[:, :2]
        dirs = torch.stack([
            torch.stack([torch.cos(yaw), torch.sin(yaw)], dim=-1),
            torch.stack([torch.cos(yaw + torch.pi / 2), torch.sin(yaw + torch.pi / 2)], dim=-1),
            torch.stack([torch.cos(yaw - torch.pi / 2), torch.sin(yaw - torch.pi / 2)], dim=-1),
        ], dim=1)  # (N, 3, 2)
        c = self.boxes[..., :2]  # (N, K, 2)
        h = self.boxes[..., 2:]  # (N, K, 2)
        o = rel.unsqueeze(1).unsqueeze(2)  # (N, 1, 1, 2)
        d = dirs.unsqueeze(2)  # (N, 3, 1, 2)
        lo, hi = c - h, c + h
        t1 = (lo.unsqueeze(1) - o) / (d + 1e-9)
        t2 = (hi.unsqueeze(1) - o) / (d + 1e-9)
        tmin = torch.minimum(t1, t2).max(dim=-1).values  # (N, 3, 2)
        tmax = torch.maximum(t1, t2).min(dim=-1).values
        hit = (tmax > torch.clamp(tmin, min=0.0)) & (tmax > 0)
        dist = torch.where(hit, torch.clamp(tmin, min=0.0), torch.full_like(tmin, 1e9))
        return dist.min(dim=-1).values.clamp(max=self.cfg.max_obs_dist)  # (N, 3)

    def _twist_to_wheels(self, v: torch.Tensor, w: torch.Tensor) -> torch.Tensor:
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
        v = (self.robot.data.root_lin_vel_w.torch[:, :2] * fwd).sum(dim=-1)
        w = self.robot.data.root_ang_vel_w.torch[:, 2]
        v = v + torch.randn_like(v) * self.cfg.odom_noise_lin
        w = w + torch.randn_like(w) * self.cfg.odom_noise_ang
        return v, w

    def _base_command(self):
        """Pure-pursuit mimic of Nav2 output toward episode goal."""
        rel = self.goal - (self.robot.data.root_pos_w.torch[:, :2] - self.scene.env_origins[:, :2])
        dist = rel.norm(dim=-1).clamp(min=1e-6)
        q0 = self.robot.data.root_quat_w.torch
        yaw = torch.atan2(2.0 * (q0[:, 0] * q0[:, 3] + q0[:, 1] * q0[:, 2]),
                          1.0 - 2.0 * (q0[:, 2] ** 2 + q0[:, 3] ** 2))
        ang = torch.atan2(rel[:, 1], rel[:, 0]) - yaw
        ang = torch.atan2(torch.sin(ang), torch.cos(ang))
        self.v_nav = torch.clamp(0.6 * dist, max=self.cfg.max_lin) * (dist > 0.3).float()
        self.w_nav = torch.clamp(2.0 * ang, -self.cfg.max_ang, self.cfg.max_ang)

    # ------------------------------------------------------------- DirectRLEnv API
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone()

    def _apply_action(self) -> None:
        applied = self._delayed.clone()
        v = torch.clamp(self.v_nav + self.actions[:, 0], -self.cfg.max_lin, self.cfg.max_lin)
        w = torch.clamp(self.w_nav + self.actions[:, 1], -self.cfg.max_ang, self.cfg.max_ang)
        self._delayed = torch.stack([v, w], dim=-1)
        targets = self._twist_to_wheels(
            torch.clamp(applied[:, 0], -self.cfg.max_lin, self.cfg.max_lin),
            torch.clamp(applied[:, 1], -self.cfg.max_ang, self.cfg.max_ang))
        self.robot.set_joint_velocity_target(targets[:, 0].unsqueeze(-1), joint_ids=self._left_idx)
        self.robot.set_joint_velocity_target(targets[:, 1].unsqueeze(-1), joint_ids=self._right_idx)

    def _get_observations(self) -> dict:
        self._base_command()
        d = self._sector_dists()
        self._last_d = d
        obs = torch.stack([self.v_nav, self.w_nav, d[:, 0], d[:, 1], d[:, 2]], dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        rel = self.goal - (self.robot.data.root_pos_w.torch[:, :2] - self.scene.env_origins[:, :2])
        dist = rel.norm(dim=-1)
        progress = self.prev_goal_dist - dist
        self.prev_goal_dist = dist.detach()
        arrived = dist < self.cfg.goal_tolerance
        collided = (self._last_d.min(dim=-1).values < self.cfg.collision_dist)
        act = (self.actions ** 2).sum(dim=-1)
        reward = progress * 2.0 - self.cfg.action_penalty * act
        reward = torch.where(arrived, reward + self.cfg.arrival_bonus, reward)
        reward = torch.where(collided, reward - self.cfg.collision_penalty, reward)
        self._arrived, self._collided = arrived, collided
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length
        terminated = getattr(self, "_arrived", time_out) | getattr(self, "_collided", time_out)
        return terminated & ~time_out, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        ids = torch.as_tensor(env_ids, device=self.device) if not isinstance(env_ids, torch.Tensor) else env_ids
        n = len(ids)
        # start at arena edge, goal on opposite side
        ang = sample_uniform(0, 6.2832, (n,), self.device)
        rad = sample_uniform(2.0, 2.8, (n,), self.device)
        start = torch.stack([rad * torch.cos(ang), rad * torch.sin(ang)], dim=-1)
        goal = -start + sample_uniform(-0.5, 0.5, (n, 2), self.device)
        root_pos = self.robot.data.default_root_state.torch[ids, :3].clone()
        root_pos[:, :2] = self.scene.env_origins[ids, :2] + start
        yaw = torch.atan2(-start[:, 1], -start[:, 0])
        root_quat = self.robot.data.default_root_state.torch[ids, 3:7].clone()
        root_quat[:, 0] = torch.cos(yaw / 2.0)
        root_quat[:, 3] = torch.sin(yaw / 2.0)
        root_quat[:, 1:3] = 0.0
        self.robot.write_root_pose_to_sim(torch.cat([root_pos, root_quat], dim=-1), ids)
        self.robot.write_root_velocity_to_sim(torch.zeros(n, 6, device=self.device), ids)
        nj = len(self._left_idx) + len(self._right_idx) + len(self._arm_idx)
        self.robot.write_joint_state_to_sim(
            torch.zeros(n, nj, device=self.device),
            torch.zeros(n, nj, device=self.device), slice(None), ids)
        hold = self.robot.data.default_joint_pos.torch[ids][:, self._arm_idx]
        self.robot.set_joint_position_target(hold, joint_ids=self._arm_idx, env_ids=ids)
        self.goal[ids] = goal
        rel = goal - start
        self.prev_goal_dist[ids] = rel.norm(dim=-1)
        lo, hi = self.cfg.radius_scale_range
        self.radius_scale[ids] = sample_uniform(lo, hi, (n,), self.device)
        self._delayed[ids] = 0.0
