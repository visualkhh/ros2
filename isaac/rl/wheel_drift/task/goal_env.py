"""goal 환경 — 목표 변위(시작 자세 기준)까지 이동하도록 바퀴 보정을 학습한다.

관측 (10):
  [gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_dv, prev_dw]

- `gx, gy`: 에피소드 시작 시 뽑은 목표 변위 (시작 자세 기준)
- `px, py, yaw`: `/odom`을 흉내낸 적분값 (시작점 0)
- `v_enc, w_enc`: 엔코더 기반 odom twist (지연·잡음 포함)
- `w_gyro`: IMU z (지연·잡음 포함)
- `prev_dv, prev_dw`: 직전 정책 action ([-1, 1])

보상은 sim의 **true 변위**로 계산한다. 정책 관측에는 true pose를 넣지 않는다.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
from isaaclab.utils.math import euler_xyz_from_quat, sample_uniform

from tasks.common import robot_spec as spec
from rl.wheel_drift.task.rl_correct_env import RlCorrectEnv, wheel_targets

if TYPE_CHECKING:
    from rl.wheel_drift.task.goal_env_cfg import GoalEnvCfg


def wrap_to_pi(a: torch.Tensor) -> torch.Tensor:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RlGoalEnv(RlCorrectEnv):
    cfg: GoalEnvCfg

    def __init__(self, cfg: GoalEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.goal = torch.zeros(n, 2, device=dev)
        self.odom = torch.zeros(n, 3, device=dev)          # [x, y, yaw] 시작점 기준
        self.odom_seen = torch.zeros(n, 3, device=dev)     # 지연된 odom (정책·컨트롤러가 봄)
        self.odom_buf = torch.zeros(n, self.cfg.obs_delay_steps[1] + 1, 3, device=dev)
        self.prev_action = torch.zeros(n, 2, device=dev)   # 정규화 action
        self.prev_dist = torch.zeros(n, device=dev)
        self.dist = torch.zeros(n, device=dev)
        self.start_xy = torch.zeros(n, 2, device=dev)
        self.start_yaw = torch.zeros(n, device=dev)

    # ------------------------------------------------------------------ 목표
    def _new_commands(self, ids: torch.Tensor):
        # goal 환경에서는 속도 명령을 뽑지 않는다. 매 스텝 nominal controller가 만든다.
        self.cmd[ids] = 0.0

    def _nominal_cmd(self) -> torch.Tensor:
        """odom(지연) 기준 목표점 추종 컨트롤러. 정책은 여기에 보정만 더한다."""
        e = self.goal - self.odom_seen[:, :2]
        dist = torch.linalg.vector_norm(e, dim=-1)
        heading = torch.atan2(e[:, 1], e[:, 0])
        herr = wrap_to_pi(heading - self.odom_seen[:, 2])

        v = torch.clamp(self.cfg.k_dist * dist, max=spec.MAX_LIN)
        w = torch.clamp(self.cfg.k_heading * herr, -spec.MAX_ANG, spec.MAX_ANG)
        turn_only = herr.abs() > self.cfg.turn_thresh
        near = dist < self.cfg.goal_tol
        v = torch.where(turn_only | near, torch.zeros_like(v), v)
        w = torch.where(near, torch.zeros_like(w), w)
        return torch.stack([v, w], dim=-1)

    # ------------------------------------------------------------------ 한 스텝
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.prev_delta = self.delta
        self.cmd = self._nominal_cmd()

        act = actions.clamp(-1.0, 1.0)
        self.prev_action = act
        moving = (self.cmd.abs().sum(dim=-1, keepdim=True) > 0).float()
        self.delta = act * self.scale * moving

        out = self.cmd + self.delta
        out = torch.stack([out[:, 0].clamp(-spec.MAX_LIN, spec.MAX_LIN),
                           out[:, 1].clamp(-spec.MAX_ANG, spec.MAX_ANG)], dim=-1)
        self.cmd_buf = torch.roll(self.cmd_buf, 1, dims=1)
        self.cmd_buf[:, 0] = out
        applied = self.cmd_buf[self._rows, self.cmd_delay]
        wheels = wheel_targets(applied[:, 0], applied[:, 1], self.r_nom, self.sep_nom,
                               spec.MAX_WHEEL)
        wheels = wheels * self.gain
        self.wheel_cmd = torch.where(wheels.abs() < self.deadband, torch.zeros_like(wheels), wheels)

    def _apply_action(self) -> None:
        alpha = 1.0 - torch.exp(-self.cfg.sim.dt / self.tau)
        self.wheel_target = self.wheel_target + alpha * (self.wheel_cmd - self.wheel_target)
        self.robot.set_joint_velocity_target(self.wheel_target, joint_ids=self._wheels)

    # ------------------------------------------------------------------ 관측·보상
    def _get_observations(self) -> dict:
        jv = torch.nan_to_num(self.robot.data.joint_vel.torch[:, self._wheels])
        v_enc = self.r_nom * (jv[:, 0] + jv[:, 1]) / 2.0
        w_enc = self.r_nom * (jv[:, 1] - jv[:, 0]) / self.sep_nom
        _, w_true = self._true_twist()

        # /odom 적분 (엔코더 기준). real 드라이버와 같은 20 Hz 적분.
        yaw = wrap_to_pi(self.odom[:, 2] + w_enc * self.policy_dt)
        self.odom[:, 0] += v_enc * torch.cos(yaw) * self.policy_dt
        self.odom[:, 1] += v_enc * torch.sin(yaw) * self.policy_dt
        self.odom[:, 2] = yaw

        self.odom_buf = torch.roll(self.odom_buf, 1, dims=1)
        self.odom_buf[:, 0] = self.odom
        self.odom_seen = self.odom_buf[self._rows, self.obs_delay]

        sens = torch.stack([
            v_enc + torch.randn_like(v_enc) * self.cfg.enc_noise,
            w_enc + torch.randn_like(w_enc) * self.cfg.enc_noise,
            w_true + torch.randn_like(w_true) * self.cfg.gyro_noise,
        ], dim=-1)
        self.sens_buf = torch.roll(self.sens_buf, 1, dims=1)
        self.sens_buf[:, 0] = sens
        seen = self.sens_buf[self._rows, self.obs_delay]

        obs = torch.cat([self.goal, self.odom_seen, seen, self.prev_action], dim=-1)
        return {"policy": obs}

    def _true_pos(self) -> torch.Tensor:
        """시작 자세 기준 true 변위 (보상 전용)."""
        d = self.robot.data.root_pos_w.torch[:, :2] - self.start_xy
        c = torch.cos(-self.start_yaw)
        s = torch.sin(-self.start_yaw)
        return torch.stack([c * d[:, 0] - s * d[:, 1],
                            s * d[:, 0] + c * d[:, 1]], dim=-1)

    def _get_rewards(self) -> torch.Tensor:
        p = self._true_pos()
        e = self.goal - p
        dist = torch.linalg.vector_norm(e, dim=-1)
        gnorm = torch.linalg.vector_norm(self.goal, dim=-1).clamp_min(1e-6)

        progress = (self.prev_dist - dist) / gnorm
        proj = (p * self.goal).sum(dim=-1) / (gnorm * gnorm)
        lateral_m = torch.linalg.vector_norm(p - proj.unsqueeze(-1) * self.goal, dim=-1)
        lateral = torch.clamp(lateral_m - self.cfg.lateral_deadband_m, min=0.0) / gnorm

        act = (self.delta ** 2).sum(dim=-1)
        jerk = ((self.delta - self.prev_delta) ** 2).sum(dim=-1)
        reward = (self.cfg.goal_progress_weight * progress
                  - self.cfg.lateral_weight * lateral
                  - self.cfg.action_penalty * act
                  - self.cfg.smooth_penalty * jerk)

        self.prev_dist = dist.detach()
        self.dist = dist.detach()
        log = self.extras.setdefault("log", {})
        log["Metrics/goal_dist"] = dist.mean().item()
        log["Metrics/goal_lateral"] = lateral_m.mean().item()
        log["Metrics/goal_progress"] = progress.mean().item()
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        dist = torch.linalg.vector_norm(self.goal - self._true_pos(), dim=-1)
        success = dist < self.cfg.goal_tol
        time_out = self.episode_length_buf >= self.max_episode_length
        self.extras.setdefault("log", {})["Metrics/success"] = success.float().mean().item()
        return success, time_out

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device)
        n, dev = len(ids), self.device

        radius = sample_uniform(*self.cfg.goal_radius_range, (n,), dev)
        theta = sample_uniform(-math.pi, math.pi, (n,), dev)
        self.goal[ids] = torch.stack([radius * torch.cos(theta),
                                      radius * torch.sin(theta)], dim=-1)

        self.odom[ids] = 0.0
        self.odom_buf[ids] = 0.0
        self.odom_seen[ids] = 0.0
        self.prev_action[ids] = 0.0
        self.cmd[ids] = 0.0
        self.prev_dist[ids] = torch.linalg.vector_norm(self.goal[ids], dim=-1)
        self.dist[ids] = self.prev_dist[ids]

        self.start_xy[ids] = self.robot.data.root_pos_w.torch[ids, :2]
        quat = self.robot.data.root_quat_w.torch[ids]
        self.start_yaw[ids] = euler_xyz_from_quat(quat)[2]
