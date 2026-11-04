"""rl_correction_waypoint — 랜덤 웨이포인트를 이탈 없이 따라가도록 기본 제어기 명령에 보정 [Δv, Δw]를 배운다.

한 정책 스텝(20 Hz) — 로봇 corrector goal 모드와 같은 순서:
  추정 자세(시작 자세 기준: 엔코더 거리 + 자이로 방향 적분)와 다음 웨이포인트 → 기본 제어기 (v, w)
  → + 정책 보정 (clamp × action_scale, 기본 제어기가 멈춤이면 0) → clip → [명령 지연]
  → 바퀴 속도(기준 반지름·간격, 8 rad/s 비례 축소) → 모터(바퀴별 이득·데드밴드·1차 지연) → 물리
추정 위치가 goal_tol 안이면 다음 웨이포인트. 관측은 실제 로봇에서 보이는 것만 (센서 지연 포함).
보상은 sim의 진짜 위치로: 경로(앞 점 → 다음 점 직선) 이탈 · 방향 · 진행 · 도착 (이동 중에도 계속 채점).
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.utils.math import sample_uniform

from envs.base_env import BaseEnv
from training.rl_correction_waypoint.markers import CommandMarkers
from training.rl_correction_waypoint.command import RlCorrectionWaypointCommand
from training.rl_correction_waypoint.ground import make_terrain, spawn_friction_patches

if TYPE_CHECKING:
    from training.rl_correction_waypoint.env_cfg import RlCorrectionWaypointEnvCfg


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(a), torch.cos(a))


class RlCorrectionWaypointEnv(BaseEnv):
    cfg: RlCorrectionWaypointEnvCfg

    def __init__(self, cfg: RlCorrectionWaypointEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        c, n, dev = self.cfg, self.num_envs, self.device   # 로봇 숫자는 장면을 만들 때 cfg.robot으로 채워져 있다
        self.command = RlCorrectionWaypointCommand(c.num_waypoints, c.waypoint_dist, c.waypoint_turn)
        self.scale = torch.tensor(c.action_scale, device=dev)
        self._rows = torch.arange(n, device=dev)
        # 웨이포인트 (시작 자세 기준)
        self.wps = torch.zeros(n, c.num_waypoints, 2, device=dev)
        self.wp_idx = torch.zeros(n, dtype=torch.long, device=dev)
        self.seg_start = torch.zeros(n, 2, device=dev)      # 지금 구간의 앞 점
        self.start = torch.zeros(n, 3, device=dev)          # 시작 자세 (월드 x, y, yaw)
        self.est = torch.zeros(n, 3, device=dev)            # 추정 자세 (시작 기준 x, y, yaw) — corrector와 같은 식
        self.prev_dist = torch.zeros(n, device=dev)         # 진짜 위치 → 목표 거리 (진행 보상)
        self.reached = torch.zeros(n, dtype=torch.bool, device=dev)
        self.reach_err = torch.zeros(n, device=dev)
        self.finished = torch.zeros(n, dtype=torch.bool, device=dev)
        self.cross = torch.zeros(n, device=dev)
        # 정책 · 모터
        self.action = torch.zeros(n, 2, device=dev)         # 정규화 [-1, 1] (관측 prev_action)
        self.prev_action = torch.zeros(n, 2, device=dev)
        self.wheel_cmd = torch.zeros(n, 2, device=dev)
        self.wheel_target = torch.zeros(n, 2, device=dev)
        # 에피소드마다 무작위
        self.tau = torch.ones(n, 2, device=dev) * 0.1
        self.gain = torch.ones(n, 2, device=dev)
        self.deadband = torch.zeros(n, 1, device=dev)
        self.gyro_bias = torch.zeros(n, device=dev)
        self.r_nom = torch.full((n,), c.wheel_radius, device=dev)
        self.sep_nom = torch.full((n,), c.wheel_sep, device=dev)
        self.cmd_delay = torch.zeros(n, dtype=torch.long, device=dev)
        self.obs_delay = torch.zeros(n, dtype=torch.long, device=dev)
        # 랜덤 범위 = 로봇 실측 응답(cfg.response_*, 장면 만들 때 robots.use가 채움) ± task 폭
        self.tau_range = (max(c.tau_min, c.response_tau_s - c.tau_width), c.response_tau_s + c.tau_width)
        self.delay_range = (max(0, c.response_delay_steps - c.delay_width), c.response_delay_steps + c.delay_width)
        self.cmd_buf = torch.zeros(n, self.delay_range[1] + 1, 2, device=dev)
        self.sens_buf = torch.zeros(n, c.obs_delay_steps[1] + 1, 3, device=dev)
        self.seen = torch.zeros(n, 3, device=dev)           # 지연 거친 [v_enc, w_enc, w_gyro]

    # ------------------------------------------------------------------ 배경 (ground.py)
    def ground_cfg(self):
        return make_terrain(self.cfg.ground)

    def ground_objects(self) -> None:
        spawn_friction_patches(self, self.cfg.ground)

    # ------------------------------------------------------------------ 자세 · 목표
    def goal(self) -> torch.Tensor:
        return self.wps[self._rows, self.wp_idx.clamp(max=self.cfg.num_waypoints - 1)]

    def true_pose(self) -> torch.Tensor:
        """진짜 자세 (시작 자세 기준 x, y, yaw) — 보상 전용."""
        d = self.robot.data
        q = d.root_quat_w.torch   # xyzw
        yaw = torch.atan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
        dx, dy = d.root_pos_w.torch[:, 0] - self.start[:, 0], d.root_pos_w.torch[:, 1] - self.start[:, 1]
        c, s = torch.cos(-self.start[:, 2]), torch.sin(-self.start[:, 2])
        return torch.stack([c * dx - s * dy, s * dx + c * dy, wrap(yaw - self.start[:, 2])], dim=-1)

    def nominal(self) -> torch.Tensor:
        """기본 제어기 (= corrector _goal_cmd): 추정 자세 → 다음 웨이포인트."""
        c = self.cfg
        e = self.goal() - self.est[:, :2]
        dist = e.norm(dim=-1)
        herr = wrap(torch.atan2(e[:, 1], e[:, 0]) - self.est[:, 2])
        v = torch.clamp(c.k_dist * dist, max=c.max_lin)
        w = torch.clamp(c.k_heading * herr, -c.max_ang, c.max_ang)
        v = torch.where((herr.abs() > c.turn_thresh) | (dist < c.goal_tol), torch.zeros_like(v), v)
        w = torch.where(dist < c.goal_tol, torch.zeros_like(w), w)
        stop = self.finished
        return torch.stack([v * ~stop, w * ~stop], dim=-1)

    # ------------------------------------------------------------------ 한 스텝
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        c = self.cfg
        self.cmd = self.nominal()
        self.prev_action = self.action
        self.action = actions.clamp(-1.0, 1.0)
        moving = (self.cmd.abs().sum(dim=-1, keepdim=True) > 0).float()
        out = self.cmd + self.action * self.scale * moving   # 기본 제어기가 멈춤이면 보정 0 (corrector correct()와 같음)
        out = torch.stack([out[:, 0].clamp(-c.max_lin, c.max_lin), out[:, 1].clamp(-c.max_ang, c.max_ang)], dim=-1)
        self.cmd_buf = torch.roll(self.cmd_buf, 1, dims=1)
        self.cmd_buf[:, 0] = out
        applied = self.cmd_buf[self._rows, self.cmd_delay]
        wheels = self.wheel_speeds(applied[:, 0], applied[:, 1], self.r_nom, self.sep_nom) * self.gain
        self.wheel_cmd = torch.where(wheels.abs() < self.deadband, torch.zeros_like(wheels), wheels)

    def _apply_action(self) -> None:
        alpha = 1.0 - torch.exp(-self.cfg.sim.dt / self.tau)
        self.wheel_target = self.wheel_target + alpha * (self.wheel_cmd - self.wheel_target)
        self.robot.set_joint_velocity_target(self.wheel_target, joint_ids=self._wheels)

    def _sense_and_advance(self) -> None:
        """물리 뒤: 센서(지연) → 추정 자세 적분 → 도착이면 다음 웨이포인트. step에서 _get_dones가 제일 먼저 불린다."""
        c, dt = self.cfg, self.policy_dt
        v_enc, w_enc = self.encoders(self.r_nom, self.sep_nom)   # 드라이버 /odom과 같은 식 (env마다 기준값 오차)
        _, w_true = self.true_twist()
        sens = torch.stack([v_enc + torch.randn_like(v_enc) * c.enc_noise,
                            w_enc + torch.randn_like(w_enc) * c.enc_noise,
                            w_true + self.gyro_bias + torch.randn_like(w_true) * c.gyro_noise], dim=-1)
        self.sens_buf = torch.roll(self.sens_buf, 1, dims=1)
        self.sens_buf[:, 0] = sens
        self.seen = self.sens_buf[self._rows, self.obs_delay]
        # 추정 자세: 거리 = 엔코더, 방향 = 자이로 (corrector가 같은 식으로 20 Hz 적분)
        yaw_mid = self.est[:, 2] + 0.5 * self.seen[:, 2] * dt
        self.est[:, 0] += self.seen[:, 0] * torch.cos(yaw_mid) * dt
        self.est[:, 1] += self.seen[:, 0] * torch.sin(yaw_mid) * dt
        self.est[:, 2] = wrap(self.est[:, 2] + self.seen[:, 2] * dt)
        # 도착 (추정 위치 기준 — 실제 로봇과 같다) → 다음 점
        tp = self.true_pose()
        self.reached = ((self.goal() - self.est[:, :2]).norm(dim=-1) < c.goal_tol) & ~self.finished
        self.reach_err = torch.where(self.reached, (self.goal() - tp[:, :2]).norm(dim=-1), torch.zeros_like(self.reach_err))
        if self.reached.any():
            r = self.reached
            self.seg_start[r] = self.goal()[r]
            self.wp_idx[r] += 1
            self.finished |= self.wp_idx >= c.num_waypoints
            self.prev_dist[r] = (self.goal()[r] - tp[r, :2]).norm(dim=-1)
        # 경로 이탈 (진짜 위치 → 앞 점~다음 점 선분)
        a, b, p = self.seg_start, self.goal(), tp[:, :2]
        ab = b - a
        t = (((p - a) * ab).sum(-1) / (ab * ab).sum(-1).clamp(min=1e-6)).clamp(0.0, 1.0)
        self.cross = (p - (a + t[:, None] * ab)).norm(dim=-1)

    # ------------------------------------------------------------------ 종료 · 보상 · 관측
    def _get_dones(self):
        self._sense_and_advance()
        time_out = self.episode_length_buf >= self.max_episode_length
        failed = self.cross > self.cfg.fail_cross
        return failed | self.finished, time_out

    def _get_rewards(self) -> torch.Tensor:
        c = self.cfg
        tp = self.true_pose()
        dist = (self.goal() - tp[:, :2]).norm(dim=-1)
        seg = self.goal() - self.seg_start
        heading_err = wrap(tp[:, 2] - torch.atan2(seg[:, 1], seg[:, 0])).abs() * (self.cmd[:, 0] > 0)
        progress = torch.where(self.reached, torch.zeros_like(dist), self.prev_dist - dist)
        self.prev_dist = torch.where(self.reached, self.prev_dist, dist)
        act = (self.action ** 2).sum(-1)
        jerk = ((self.action - self.prev_action) ** 2).sum(-1)
        r = (-c.w_cross * self.cross - c.w_heading * heading_err + c.w_progress * progress
             + self.reached * (c.reach_bonus - c.w_reach_err * self.reach_err)
             - c.fail_penalty * (self.cross > c.fail_cross) - c.action_penalty * act - c.smooth_penalty * jerk)
        log = self.extras.setdefault("log", {})
        log["Metrics/cross_track"] = self.cross.mean().item()
        log["Metrics/waypoints_done"] = self.wp_idx.float().mean().item()
        return r

    def _get_observations(self) -> dict:
        """[gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_a0, prev_a1] — corrector goal 모드와 같은 순서."""
        return {"policy": torch.cat([self.goal(), self.est, self.seen, self.action], dim=-1)}

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)   # 지형 칸 가운데 · 방향 랜덤 (cfg.random_yaw) · 바퀴 0
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device)
        n, dev, c = len(ids), self.device, self.cfg
        self.start[ids] = self.reset_pose[ids]   # BaseEnv가 방금 놓은 자세 (월드 x, y, yaw)
        self.wps[ids] = self.command.sample(n, dev)
        self.wp_idx[ids] = 0
        self.seg_start[ids] = 0.0
        self.est[ids] = 0.0
        self.prev_dist[ids] = self.wps[ids, 0].norm(dim=-1)
        self.finished[ids] = False
        self.reached[ids] = False
        self.cross[ids] = 0.0
        self.action[ids] = 0.0
        self.prev_action[ids] = 0.0
        self.tau[ids] = sample_uniform(*self.tau_range, (n, 2), dev)
        self.gain[ids] = sample_uniform(*c.gain_range, (n, 2), dev) * torch.tensor(c.response_gain_lr, device=dev)
        self.deadband[ids] = c.response_deadband + sample_uniform(*c.deadband_range, (n, 1), dev)
        self.r_nom[ids] = c.wheel_radius * (1 + sample_uniform(-c.radius_nominal_err, c.radius_nominal_err, (n,), dev))
        self.sep_nom[ids] = c.wheel_sep * (1 + sample_uniform(-c.sep_nominal_err, c.sep_nominal_err, (n,), dev))
        self.gyro_bias[ids] = sample_uniform(-c.gyro_bias, c.gyro_bias, (n,), dev)
        self.cmd_delay[ids] = torch.randint(self.delay_range[0], self.delay_range[1] + 1, (n,), device=dev)
        self.obs_delay[ids] = torch.randint(c.obs_delay_steps[0], c.obs_delay_steps[1] + 1, (n,), device=dev)
        self.cmd_buf[ids] = 0.0
        self.sens_buf[ids] = 0.0
        self.seen[ids] = 0.0
        self.wheel_target[ids] = 0.0
        self.wheel_cmd[ids] = 0.0

    # ------------------------------------------------------------------ 화면 (cfg.debug_vis일 때 BaseEnv가 부른다)
    def debug_vis_setup(self, visible: bool) -> None:
        if visible and getattr(self, "markers", None) is None:
            self.markers = CommandMarkers()
        if getattr(self, "markers", None) is not None:
            self.markers.set_visible(visible)

    def debug_vis_draw(self) -> None:
        self.markers.draw(self, self.vis_path())

    def vis_path(self):
        """노란 점 = 시작 → 웨이포인트들을 잇는 경로 (월드). 막대는 BaseEnv."""
        if not hasattr(self, "wps"):
            return None
        pts = torch.cat([torch.zeros(self.num_envs, 1, 2, device=self.device), self.wps], dim=1)   # [n, K+1, 2]
        k = torch.linspace(0, 1, 12, device=self.device)[:-1]
        seg = pts[:, :-1, None, :] + (pts[:, 1:, None, :] - pts[:, :-1, None, :]) * k[None, None, :, None]   # [n, K, 11, 2]
        seg = seg.reshape(self.num_envs, -1, 2)
        c, s = torch.cos(self.start[:, 2])[:, None], torch.sin(self.start[:, 2])[:, None]
        x = self.start[:, 0:1] + c * seg[..., 0] - s * seg[..., 1]
        y = self.start[:, 1:2] + s * seg[..., 0] + c * seg[..., 1]
        z = self.robot.data.root_pos_w.torch[:, 2:3].expand_as(x)
        return torch.stack([x, y, z], dim=-1).reshape(-1, 3)

