"""rl_nav_correction — 명령 (v, w)를 몸체가 그대로 따라가도록 보정 [Δv, Δw]를 배운다 (nav2 컨트롤러 뒤 보정 노드용).

한 정책 스텝(20 Hz) — ml rl_nav_correction 노드와 같은 순서:
  명령 (v, w) (command.py — 0.5~2 s마다 바뀜) + 정책 보정 (clamp × action_scale, 명령이 0이면 0) → clip → [명령 지연]
  → 바퀴 속도(기준 반지름·간격, 8 rad/s 비례 축소) → 모터(바퀴별 이득·데드밴드·1차 지연) → 물리
관측 = [v_cmd, w_cmd, v_odom, w_odom, w_gyro, t_change] — 노드와 같은 순서 · 원래 단위, 센서 지연 포함 (실기기에서 보이는 것만).
다음에 적용될 명령을 관측에 넣는다 (노드는 명령이 들어오는 그 순간 추론한다). 보상은 진짜 몸체 속도가 그 명령을 따라가는 정도.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.utils.math import sample_uniform

from envs.base_env import BaseEnv
from training.rl_nav_correction.markers import CommandMarkers
from training.rl_nav_correction.command import RlNavCorrectionCommand
from training.rl_nav_correction.ground import make_terrain, spawn_friction_patches

if TYPE_CHECKING:
    from training.rl_nav_correction.env_cfg import RlNavCorrectionEnvCfg


class RlNavCorrectionEnv(BaseEnv):
    cfg: RlNavCorrectionEnvCfg

    def __init__(self, cfg: RlNavCorrectionEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        c, n, dev = self.cfg, self.num_envs, self.device   # 로봇 숫자는 장면을 만들 때 cfg.robot으로 채워져 있다
        self.command = RlNavCorrectionCommand(c.max_lin, c.max_ang, c.command_hold)
        self.scale = torch.tensor(c.action_scale, device=dev)
        self._rows = torch.arange(n, device=dev)
        # 명령
        self.cmd = torch.zeros(n, 2, device=dev)            # 다음에 적용될 명령 (관측)
        self.cmd_applied = torch.zeros(n, 2, device=dev)    # 방금 적용된 명령 (보상)
        self.hold_left = torch.zeros(n, dtype=torch.long, device=dev)
        self.since = torch.zeros(n, device=dev)             # 명령이 바뀐 뒤 스텝
        # 정책 · 모터
        self.action = torch.zeros(n, 2, device=dev)         # 정규화 [-1, 1]
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

    # ------------------------------------------------------------------ 한 스텝
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        c = self.cfg
        self.cmd_applied = self.cmd.clone()
        self.prev_action = self.action
        self.action = actions.clamp(-1.0, 1.0)
        moving = (self.cmd.abs().sum(dim=-1, keepdim=True) > 0).float()
        out = self.cmd + self.action * self.scale * moving   # 명령이 0이면 보정 0 (노드 apply_delta와 같음)
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
        self.hold_joints()

    def _sense_and_advance(self) -> None:
        """물리 뒤: 센서(지연) → 명령 시간 지나면 다음 명령. step에서 _get_dones가 제일 먼저 불린다."""
        c = self.cfg
        v_enc, w_enc = self.encoders(self.r_nom, self.sep_nom)   # 드라이버 /odom과 같은 식 (env마다 기준값 오차)
        _, w_true = self.true_twist()
        sens = torch.stack([v_enc + torch.randn_like(v_enc) * c.enc_noise,
                            w_enc + torch.randn_like(w_enc) * c.enc_noise,
                            w_true + self.gyro_bias + torch.randn_like(w_true) * c.gyro_noise], dim=-1)
        self.sens_buf = torch.roll(self.sens_buf, 1, dims=1)
        self.sens_buf[:, 0] = sens
        self.seen = self.sens_buf[self._rows, self.obs_delay]
        self.since += 1
        self.hold_left -= 1
        ch = self.hold_left <= 0
        if ch.any():
            k = int(ch.sum())
            self.cmd[ch] = self.command.sample(k, self.device)
            self.hold_left[ch] = self.command.hold_steps(k, self.policy_dt, self.device)
            self.since[ch] = 0.0

    # ------------------------------------------------------------------ 종료 · 보상 · 관측
    def _get_dones(self):
        self._sense_and_advance()
        time_out = self.episode_length_buf >= self.max_episode_length
        return torch.zeros_like(time_out), time_out

    def _get_rewards(self) -> torch.Tensor:
        c = self.cfg
        v, w = self.true_twist()
        ev = (v - self.cmd_applied[:, 0]) / c.max_lin
        ew = (w - self.cmd_applied[:, 1]) / c.max_ang
        act = (self.action ** 2).sum(-1)
        jerk = ((self.action - self.prev_action) ** 2).sum(-1)
        r = -c.w_track_v * ev.abs() - c.w_track_w * ew.abs() - c.action_penalty * act - c.smooth_penalty * jerk
        log = self.extras.setdefault("log", {})
        log["Metrics/track_err_v"] = (ev.abs() * c.max_lin).mean().item()
        log["Metrics/track_err_w"] = (ew.abs() * c.max_ang).mean().item()
        return r

    def _get_observations(self) -> dict:
        """[v_cmd, w_cmd, v_odom, w_odom, w_gyro, t_change] — rl_nav_correction 노드와 같은 순서 · 단위."""
        t_change = (self.since * self.policy_dt).clamp(max=self.cfg.t_change_max)[:, None]
        return {"policy": torch.cat([self.cmd, self.seen, t_change], dim=-1)}

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)   # 지형 칸 가운데 · 방향 랜덤 (cfg.random_yaw) · 바퀴 0
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device)
        n, dev, c = len(ids), self.device, self.cfg
        self.cmd[ids] = self.command.sample(n, dev)
        self.cmd_applied[ids] = 0.0
        self.hold_left[ids] = self.command.hold_steps(n, self.policy_dt, dev)
        self.since[ids] = 0.0
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
        self.markers.draw(self)   # 초록 = 명령 · 파랑 = 실제 몸체 (경로 점 없음)
