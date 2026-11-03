"""rl_correct v2 — 명령(v, w)대로 움직이도록 보정량 [dV, dW]를 배운다.

한 정책 스텝(20 Hz):
  명령(계단식으로 바뀜) + 보정 → clip → [cmd 지연] → 바퀴 속도(기준 반지름·간격, 8 rad/s 비례 축소 = opencr_driver)
  → 모터(바퀴별 이득·데드밴드·1차 지연) → 물리
관측은 실제 로봇에서 보이는 것만: 엔코더 v/w(기준값으로 계산 = 드라이버 /odom) · 자이로 w(/imu) — [obs 지연].
보상은 진짜 몸체 속도로 채점 (학습 중엔 정답을 앎).
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.utils.math import sample_uniform

from envs.base_env import BaseEnv
from robots.beebot import config as beebot

if TYPE_CHECKING:
    from training.rl_correction_wheel_drift.rl_correct_env_cfg import RlCorrectEnvCfg

# 텔레오퍼 키 → (전후, 회전) 방향 (tool/cli/op/bus.py TELEOP_MOVES와 같음)
TELEOP_MOVES = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1))



class RlCorrectEnv(BaseEnv):
    cfg: RlCorrectEnvCfg

    def __init__(self, cfg: RlCorrectEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.scale = torch.tensor(self.cfg.action_scale, device=dev)
        self.hold = torch.zeros(n, device=dev)
        self.hold_total = torch.zeros(n, device=dev)       # 이번 명령의 유지 시간 (목표 경로 그림용)
        self.t_change = torch.zeros(n, device=dev)
        self.delta = torch.zeros(n, 2, device=dev)
        self.prev_delta = torch.zeros(n, 2, device=dev)
        self.wheel_cmd = torch.zeros(n, 2, device=dev)     # 모터에 들어가는 명령 [rad/s]
        self.wheel_target = torch.zeros(n, 2, device=dev)  # 1차 지연을 거친 실제 목표
        # 에피소드마다 무작위
        self.tau = torch.ones(n, 2, device=dev) * 0.1
        self.gain = torch.ones(n, 2, device=dev)
        self.deadband = torch.zeros(n, 1, device=dev)
        self.gyro_bias = torch.zeros(n, device=dev)        # 자이로 영점 오차 [rad/s] (에피소드 동안 고정)
        self.r_nom = torch.full((n,), beebot.WHEEL_RADIUS, device=dev)
        self.sep_nom = torch.full((n,), beebot.WHEEL_SEP, device=dev)
        self.cmd_delay = torch.zeros(n, dtype=torch.long, device=dev)
        self.obs_delay = torch.zeros(n, dtype=torch.long, device=dev)
        self.cmd_buf = torch.zeros(n, self.cfg.cmd_delay_steps[1] + 1, 2, device=dev)
        self.sens_buf = torch.zeros(n, self.cfg.obs_delay_steps[1] + 1, 3, device=dev)
        self._rows = torch.arange(n, device=dev)


    # ------------------------------------------------------------------ 화면 표시 (--watch · --snap)
    def vis_path(self):
        """노란 점 = 명령이 바뀐 순간의 자세에서 그 명령대로 유지 시간 동안 갈 길 (막대는 BaseEnv)."""
        if not hasattr(self, "t_change"):
            return None
        if getattr(self, "_vis_start", None) is None:
            self._vis_start = torch.zeros(self.num_envs, 4, device=self.device)   # (x, y, z, yaw)
            self._vis_t_prev = torch.full((self.num_envs,), 1e9, device=self.device)
        d = self.robot.data
        pos, yaw = d.root_pos_w.torch, self.markers.yaw_of(d.root_quat_w.torch)
        changed = self.t_change < self._vis_t_prev      # 명령이 바뀐(또는 리셋된) env
        self._vis_start[changed] = torch.cat([pos[changed], yaw[changed, None]], dim=-1)
        self._vis_t_prev = self.t_change.clone()
        moving = self.cmd.abs().sum(dim=-1) > 0
        if not moving.any():
            return None
        return self.markers.arc(self._vis_start[moving], self.cmd[moving, 0], self.cmd[moving, 1], self.hold_total[moving])

    # ------------------------------------------------------------------ 명령
    def _new_commands(self, ids: torch.Tensor):
        k, dev = len(ids), self.device
        u = torch.rand(k, device=dev)
        speed = sample_uniform(0.10, beebot.MAX_LIN, (k,), dev)
        turn = sample_uniform(0.40, beebot.MAX_ANG, (k,), dev)
        moves = torch.tensor(TELEOP_MOVES, dtype=torch.float32, device=dev)[torch.randint(0, len(TELEOP_MOVES), (k,), device=dev)]
        # 텔레오퍼: 사선은 설정 속도 + 반경 DIAG_RADIUS 원호, 제자리 회전은 회전 속도
        v = moves[:, 0] * speed
        w = torch.where(moves[:, 0] != 0, moves[:, 1] * speed / beebot.DIAG_RADIUS, moves[:, 1] * turn)
        smooth = (u >= self.cfg.p_teleop) & (u < self.cfg.p_teleop + self.cfg.p_smooth)
        v = torch.where(smooth, sample_uniform(0.0, beebot.MAX_LIN, (k,), dev), v)
        w = torch.where(smooth, sample_uniform(-1.0, 1.0, (k,), dev), w)
        stop = u >= self.cfg.p_teleop + self.cfg.p_smooth
        v, w = v * (~stop), w * (~stop)
        self.cmd[ids] = torch.stack([v, w], dim=-1)
        self.hold[ids] = sample_uniform(*self.cfg.hold_range, (k,), dev)
        self.hold_total[ids] = self.hold[ids]
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
        out = torch.stack([out[:, 0].clamp(-beebot.MAX_LIN, beebot.MAX_LIN),
                           out[:, 1].clamp(-beebot.MAX_ANG, beebot.MAX_ANG)], dim=-1)
        self.cmd_buf = torch.roll(self.cmd_buf, 1, dims=1)
        self.cmd_buf[:, 0] = out
        applied = self.cmd_buf[self._rows, self.cmd_delay]
        wheels = self.wheel_speeds(applied[:, 0], applied[:, 1], self.r_nom, self.sep_nom)
        wheels = wheels * self.gain
        self.wheel_cmd = torch.where(wheels.abs() < self.deadband, torch.zeros_like(wheels), wheels)

    def _apply_action(self) -> None:
        alpha = 1.0 - torch.exp(-self.cfg.sim.dt / self.tau)
        self.wheel_target = self.wheel_target + alpha * (self.wheel_cmd - self.wheel_target)
        self.robot.set_joint_velocity_target(self.wheel_target, joint_ids=self._wheels)

    # ------------------------------------------------------------------ 관측·보상
    def _get_observations(self) -> dict:
        v_enc, w_enc = self.encoders(self.r_nom, self.sep_nom)   # 드라이버 /odom과 같은 식 (env마다 기준값 오차)
        _, w_true = self.true_twist()
        sens = torch.stack([v_enc + torch.randn_like(v_enc) * self.cfg.enc_noise,
                            w_enc + torch.randn_like(w_enc) * self.cfg.enc_noise,
                            w_true + self.gyro_bias + torch.randn_like(w_true) * self.cfg.gyro_noise], dim=-1)
        self.sens_buf = torch.roll(self.sens_buf, 1, dims=1)
        self.sens_buf[:, 0] = sens
        seen = self.sens_buf[self._rows, self.obs_delay]
        obs = torch.cat([self.cmd, seen, self.t_change.clamp(max=2.0).unsqueeze(-1)], dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        v, w = self.true_twist()
        err = (self.cmd[:, 0] - v).abs() + self.cfg.ang_weight * (self.cmd[:, 1] - w).abs()
        act = (self.delta ** 2).sum(dim=-1)
        jerk = ((self.delta - self.prev_delta) ** 2).sum(dim=-1)
        self.extras.setdefault("log", {})["Metrics/track_err"] = err.mean().item()
        return -err - self.cfg.action_penalty * act - self.cfg.smooth_penalty * jerk

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)   # 지형 칸 가운데 · 방향 랜덤 (cfg.random_yaw) · 바퀴 0
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device)
        n, dev, c = len(ids), self.device, self.cfg

        self.tau[ids] = sample_uniform(*c.tau_range, (n, 2), dev)
        self.gain[ids] = sample_uniform(*c.gain_range, (n, 2), dev) * torch.tensor(c.gain_center, device=dev)
        self.deadband[ids] = sample_uniform(*c.deadband_range, (n, 1), dev)
        self.r_nom[ids] = beebot.WHEEL_RADIUS * (1 + sample_uniform(-c.radius_nominal_err, c.radius_nominal_err, (n,), dev))
        self.sep_nom[ids] = beebot.WHEEL_SEP * (1 + sample_uniform(-c.sep_nominal_err, c.sep_nominal_err, (n,), dev))
        self.gyro_bias[ids] = sample_uniform(-c.gyro_bias, c.gyro_bias, (n,), dev)
        self.cmd_delay[ids] = torch.randint(c.cmd_delay_steps[0], c.cmd_delay_steps[1] + 1, (n,), device=dev)
        self.obs_delay[ids] = torch.randint(c.obs_delay_steps[0], c.obs_delay_steps[1] + 1, (n,), device=dev)
        self.cmd_buf[ids] = 0.0
        self.sens_buf[ids] = 0.0
        self.wheel_target[ids] = 0.0
        self.wheel_cmd[ids] = 0.0
        self.delta[ids] = 0.0
        self.prev_delta[ids] = 0.0
        self._new_commands(ids)
