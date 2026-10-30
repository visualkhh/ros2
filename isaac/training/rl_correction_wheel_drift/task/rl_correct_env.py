"""rl_correct v2 — 명령(v, w)대로 움직이도록 보정량 [dV, dW]를 배운다.

한 정책 스텝(20 Hz):
  명령(계단식으로 바뀜) + 보정 → clip → [cmd 지연] → 바퀴 속도(기준 반지름·간격, 8 rad/s 비례 축소 = opencr_driver)
  → 모터(바퀴별 이득·데드밴드·1차 지연) → 물리
관측은 실제 로봇에서 보이는 것만: 엔코더 v/w(기준값으로 계산 = 드라이버 /odom) · 자이로 w(/imu) — [obs 지연].
보상은 진짜 몸체 속도로 채점 (학습 중엔 정답을 앎).
"""
from __future__ import annotations

import math
import random
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab import cloner
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform

from tasks.common import robot_spec as spec

if TYPE_CHECKING:
    from training.rl_correction_wheel_drift.task.rl_correct_env_cfg import RlCorrectEnvCfg

# 텔레오퍼 키 → (전후, 회전) 방향 (tool/cli/op/bus.py TELEOP_MOVES와 같음)
TELEOP_MOVES = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1))


def wheel_targets(v, w, r, sep, max_wheel):
    """Twist → (좌, 우) rad/s. 한 바퀴라도 넘으면 두 바퀴를 같은 비율로 (opencr_driver.wheel_speeds와 같음)."""
    wl = (v - w * sep / 2.0) / r
    wr = (v + w * sep / 2.0) / r
    wheels = torch.stack([wl, wr], dim=-1)
    peak = wheels.abs().max(dim=-1, keepdim=True).values
    return wheels * torch.clamp(max_wheel / (peak + 1e-9), max=1.0)


class RlCorrectEnv(DirectRLEnv):
    cfg: RlCorrectEnvCfg

    def __init__(self, cfg: RlCorrectEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._wheels, _ = self.robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
        n, dev = self.num_envs, self.device
        self.policy_dt = self.cfg.sim.dt * self.cfg.decimation
        self.scale = torch.tensor(self.cfg.action_scale, device=dev)
        self.cmd = torch.zeros(n, 2, device=dev)
        self.hold = torch.zeros(n, device=dev)
        self.t_change = torch.zeros(n, device=dev)
        self.delta = torch.zeros(n, 2, device=dev)
        self.prev_delta = torch.zeros(n, 2, device=dev)
        self.wheel_cmd = torch.zeros(n, 2, device=dev)     # 모터에 들어가는 명령 [rad/s]
        self.wheel_target = torch.zeros(n, 2, device=dev)  # 1차 지연을 거친 실제 목표
        # 에피소드마다 무작위
        self.tau = torch.ones(n, 2, device=dev) * 0.1
        self.gain = torch.ones(n, 2, device=dev)
        self.deadband = torch.zeros(n, 1, device=dev)
        self.r_nom = torch.full((n,), spec.WHEEL_RADIUS, device=dev)
        self.sep_nom = torch.full((n,), spec.WHEEL_SEP, device=dev)
        self.cmd_delay = torch.zeros(n, dtype=torch.long, device=dev)
        self.obs_delay = torch.zeros(n, dtype=torch.long, device=dev)
        self.cmd_buf = torch.zeros(n, self.cfg.cmd_delay_steps[1] + 1, 2, device=dev)
        self.sens_buf = torch.zeros(n, self.cfg.obs_delay_steps[1] + 1, 3, device=dev)
        self._rows = torch.arange(n, device=dev)

    # ------------------------------------------------------------------ scene
    def _setup_ground(self):
        c = self.cfg
        if getattr(c, "randomize_friction", False):
            # env마다 ground slab을 따로 만들고, 그 위에 국소 마찰 패치를 뿌린다.
            spacing = self.scene.cfg.env_spacing
            size = max(1.0, spacing - 0.1)
            pos = cloner.grid_transforms(self.scene.num_envs, spacing, device=self.device)[0]

            slip_mat = sim_utils.RigidBodyMaterialCfg(
                static_friction=c.slippery_friction[0], dynamic_friction=c.slippery_friction[1],
                restitution=0.0, friction_combine_mode="average",
            )
            slip_mat.func("/World/Materials/friction_slippery", slip_mat)
            stick_mat = sim_utils.RigidBodyMaterialCfg(
                static_friction=c.sticky_friction[0], dynamic_friction=c.sticky_friction[1],
                restitution=0.0, friction_combine_mode="average",
            )
            stick_mat.func("/World/Materials/friction_sticky", stick_mat)

            for i, (x, y, _z) in enumerate(pos.detach().cpu().tolist()):
                static = random.uniform(*c.ground_static_range)
                dynamic = random.uniform(*c.ground_dynamic_range)
                mat = sim_utils.RigidBodyMaterialCfg(
                    static_friction=static, dynamic_friction=dynamic,
                    restitution=0.0, friction_combine_mode="average",
                )
                cfg = sim_utils.CuboidCfg(
                    size=(size, size, 0.1),
                    collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                    physics_material=mat,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.45, 0.45)),
                )
                cfg.func(f"/World/grounds/ground_{i}", cfg, translation=(x, y, -0.05))

                # 파랑 = 미끄럼, 주황 = 끈적. 얇은 판이라 바퀴가 그대로 지나간다.
                for j in range(int(c.friction_patch_count)):
                    angle = (2.0 * math.pi * j / max(1, int(c.friction_patch_count))
                             + random.uniform(-0.35, 0.35))
                    radius = random.uniform(*c.friction_patch_radius)
                    pw = random.uniform(*c.friction_patch_size)
                    pd = random.uniform(*c.friction_patch_size)
                    sticky = random.random() < 0.5
                    mat_path = ("/World/Materials/friction_sticky" if sticky
                                else "/World/Materials/friction_slippery")
                    color = (1.0, 0.45, 0.10) if sticky else (0.20, 0.50, 1.0)
                    patch = sim_utils.CuboidCfg(
                        size=(pw, pd, 0.01),
                        collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=color, roughness=0.8),
                    )
                    path = f"/World/patches/patch_{i}_{j}"
                    patch.func(path, patch,
                               translation=(x + radius * math.cos(angle),
                                            y + radius * math.sin(angle), -0.004))
                    sim_utils.bind_physics_material(f"{path}/geometry/mesh", mat_path)
        else:
            g = c.ground_friction
            mat = sim_utils.RigidBodyMaterialCfg(
                static_friction=g[0], dynamic_friction=g[1],
                restitution=0.0, friction_combine_mode="average",
            )
            spawn_ground_plane(prim_path="/World/ground",
                               cfg=GroundPlaneCfg(physics_material=mat))

    def _bind_robot_materials(self):
        from pxr import UsdPhysics

        c = self.cfg
        wheel_mat = sim_utils.RigidBodyMaterialCfg(
            static_friction=c.wheel_friction[0], dynamic_friction=c.wheel_friction[1],
            restitution=0.0, friction_combine_mode="average",
        )
        wheel_mat.func("/World/Materials/wheel", wheel_mat)
        caster_mat = sim_utils.RigidBodyMaterialCfg(
            static_friction=c.caster_friction[0], dynamic_friction=c.caster_friction[1],
            restitution=0.0, friction_combine_mode="min",
        )
        caster_mat.func("/World/Materials/caster", caster_mat)
        for prim in self.scene.stage.Traverse():
            path = str(prim.GetPath())
            if not path.startswith("/World/envs/env_0/Robot") or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if "wheel" in path:
                sim_utils.bind_physics_material(path, "/World/Materials/wheel")
            elif "caster" in path:
                sim_utils.bind_physics_material(path, "/World/Materials/caster")

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self._setup_ground()
        self._bind_robot_materials()
        src, dest = "/World/envs/env_0", "/World/envs/env_{}"
        pos = cloner.grid_transforms(self.scene.num_envs, self.scene.cfg.env_spacing, device=self.device)[0]
        plan = cloner.clone_plan_from_env_0(src, dest, self.scene.num_envs, self.device, pos, ("/World/ground",))
        cloner.replicate(plan, stage=self.scene.stage)
        if "physx" in self.scene.physics_backend:
            self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations["robot"] = self.robot
        light_cfg = sim_utils.DistantLightCfg(intensity=2000.0, color=(1.0, 1.0, 1.0))
        light_cfg.func("/World/Light", light_cfg)

    # ------------------------------------------------------------------ 명령
    def _new_commands(self, ids: torch.Tensor):
        k, dev = len(ids), self.device
        u = torch.rand(k, device=dev)
        speed = sample_uniform(0.10, spec.MAX_LIN, (k,), dev)
        turn = sample_uniform(0.40, spec.MAX_ANG, (k,), dev)
        moves = torch.tensor(TELEOP_MOVES, dtype=torch.float32, device=dev)[torch.randint(0, len(TELEOP_MOVES), (k,), device=dev)]
        # 텔레오퍼: 사선은 설정 속도 + 반경 DIAG_RADIUS 원호, 제자리 회전은 회전 속도
        v = moves[:, 0] * speed
        w = torch.where(moves[:, 0] != 0, moves[:, 1] * speed / spec.DIAG_RADIUS, moves[:, 1] * turn)
        smooth = (u >= self.cfg.p_teleop) & (u < self.cfg.p_teleop + self.cfg.p_smooth)
        v = torch.where(smooth, sample_uniform(0.0, spec.MAX_LIN, (k,), dev), v)
        w = torch.where(smooth, sample_uniform(-1.0, 1.0, (k,), dev), w)
        stop = u >= self.cfg.p_teleop + self.cfg.p_smooth
        v, w = v * (~stop), w * (~stop)
        self.cmd[ids] = torch.stack([v, w], dim=-1)
        self.hold[ids] = sample_uniform(*self.cfg.hold_range, (k,), dev)
        self.t_change[ids] = 0.0

    # ------------------------------------------------------------------ 한 스텝
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.hold -= self.policy_dt
        self.t_change += self.policy_dt
        due = (self.hold <= 0).nonzero(as_tuple=False).squeeze(-1)
        if len(due):
            self._new_commands(due)
        self.prev_delta = self.delta
        moving = (self.cmd.abs().sum(dim=-1, keepdim=True) > 0).float()
        self.delta = actions.clamp(-1.0, 1.0) * self.scale * moving  # 정지 명령엔 보정 0 (ROS corrector와 같은 규칙 — 제자리 기어감 방지)
        out = self.cmd + self.delta
        out = torch.stack([out[:, 0].clamp(-spec.MAX_LIN, spec.MAX_LIN),
                           out[:, 1].clamp(-spec.MAX_ANG, spec.MAX_ANG)], dim=-1)
        self.cmd_buf = torch.roll(self.cmd_buf, 1, dims=1)
        self.cmd_buf[:, 0] = out
        applied = self.cmd_buf[self._rows, self.cmd_delay]
        wheels = wheel_targets(applied[:, 0], applied[:, 1], self.r_nom, self.sep_nom, spec.MAX_WHEEL)
        wheels = wheels * self.gain
        self.wheel_cmd = torch.where(wheels.abs() < self.deadband, torch.zeros_like(wheels), wheels)

    def _apply_action(self) -> None:
        alpha = 1.0 - torch.exp(-self.cfg.sim.dt / self.tau)
        self.wheel_target = self.wheel_target + alpha * (self.wheel_cmd - self.wheel_target)
        self.robot.set_joint_velocity_target(self.wheel_target, joint_ids=self._wheels)

    # ------------------------------------------------------------------ 관측·보상
    def _true_twist(self) -> tuple[torch.Tensor, torch.Tensor]:
        # 몸체 기준 속도 (Lab 6.1 쿼터니언은 xyzw — 손으로 풀지 않고 _b 값을 씀)
        v = self.robot.data.root_lin_vel_b.torch[:, 0]
        w = self.robot.data.root_ang_vel_b.torch[:, 2]
        return torch.nan_to_num(v), torch.nan_to_num(w)

    def _get_observations(self) -> dict:
        jv = torch.nan_to_num(self.robot.data.joint_vel.torch[:, self._wheels])
        v_enc = self.r_nom * (jv[:, 0] + jv[:, 1]) / 2.0          # 드라이버 /odom과 같은 식 (기준값)
        w_enc = self.r_nom * (jv[:, 1] - jv[:, 0]) / self.sep_nom
        _, w_true = self._true_twist()
        sens = torch.stack([v_enc + torch.randn_like(v_enc) * self.cfg.enc_noise,
                            w_enc + torch.randn_like(w_enc) * self.cfg.enc_noise,
                            w_true + torch.randn_like(w_true) * self.cfg.gyro_noise], dim=-1)
        self.sens_buf = torch.roll(self.sens_buf, 1, dims=1)
        self.sens_buf[:, 0] = sens
        seen = self.sens_buf[self._rows, self.obs_delay]
        obs = torch.cat([self.cmd, seen, self.t_change.clamp(max=2.0).unsqueeze(-1)], dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        v, w = self._true_twist()
        err = (self.cmd[:, 0] - v).abs() + self.cfg.ang_weight * (self.cmd[:, 1] - w).abs()
        act = (self.delta ** 2).sum(dim=-1)
        jerk = ((self.delta - self.prev_delta) ** 2).sum(dim=-1)
        self.extras.setdefault("log", {})["Metrics/track_err"] = err.mean().item()
        return -err - self.cfg.action_penalty * act - self.cfg.smooth_penalty * jerk

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length
        return torch.zeros_like(time_out), time_out

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device)
        n, dev, c = len(ids), self.device, self.cfg
        root = self.robot.data.default_root_state.torch[ids].clone()
        root[:, :2] += self.scene.env_origins[ids, :2]
        yaw = sample_uniform(-math.pi, math.pi, (n,), dev)
        root[:, 3:7] = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
        self.robot.write_root_pose_to_sim(root[:, :7], ids)
        self.robot.write_root_velocity_to_sim(torch.zeros(n, 6, device=dev), ids)
        zeros = torch.zeros(n, len(self._wheels), device=dev)
        self.robot.write_joint_state_to_sim(zeros, zeros, self._wheels, ids)

        self.tau[ids] = sample_uniform(*c.tau_range, (n, 2), dev)
        self.gain[ids] = sample_uniform(*c.gain_range, (n, 2), dev)
        self.deadband[ids] = sample_uniform(*c.deadband_range, (n, 1), dev)
        self.r_nom[ids] = spec.WHEEL_RADIUS * (1 + sample_uniform(-c.radius_nominal_err, c.radius_nominal_err, (n,), dev))
        self.sep_nom[ids] = spec.WHEEL_SEP * (1 + sample_uniform(-c.sep_nominal_err, c.sep_nominal_err, (n,), dev))
        self.cmd_delay[ids] = torch.randint(c.cmd_delay_steps[0], c.cmd_delay_steps[1] + 1, (n,), device=dev)
        self.obs_delay[ids] = torch.randint(c.obs_delay_steps[0], c.obs_delay_steps[1] + 1, (n,), device=dev)
        self.cmd_buf[ids] = 0.0
        self.sens_buf[ids] = 0.0
        self.wheel_target[ids] = 0.0
        self.wheel_cmd[ids] = 0.0
        self.delta[ids] = 0.0
        self.prev_delta[ids] = 0.0
        self._new_commands(ids)
