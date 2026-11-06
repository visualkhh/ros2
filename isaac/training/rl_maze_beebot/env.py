"""RlMazeBeebotEnv — beebot 미로 주행. rl_maze env를 그대로 쓰고 센서만 point cloud 가상 스캔에 맞춘다.

  beebot `rl_nav_controller.cpp`의 points 경로:
    /pole_front_cam/depth/color/points_filtered → base 변환 → 36방향 빈의 최소거리.
  여기서는 미로 격자 raycast를 같은 36방향에 대해 수행하되, pole_front_cam FOV(55°) 밖은
  lidar_max로 둔다 (카메라에 안 보이는 방향 = 점이 없음 = plugin의 best 기본값).
  관측은 51차원 — inflation 입력 없이 raw 스캔 (beebot plugin과 같은 계약).
"""
from __future__ import annotations

import math

import torch

from training.rl_maze.env import RlMazeEnv
from training.rl_maze_beebot.env_cfg import RlMazeBeebotEnvCfg


class RlMazeBeebotEnv(RlMazeEnv):
    cfg: RlMazeBeebotEnvCfg

    def lidar(self, xy: torch.Tensor, yaw: torch.Tensor,
              angles: torch.Tensor | None = None) -> torch.Tensor:
        raw = super().lidar(xy, yaw, angles)
        a = self.ray_ang if angles is None else angles
        rel = torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi
        half = math.radians(float(self.cfg.camera_fov_deg)) / 2.0
        mask = rel.abs() <= half
        return torch.where(mask[None, :], raw,
                           torch.full_like(raw, float(self.cfg.lidar_max)))

    def _get_observations(self) -> dict:
        c = self.cfg
        xy, yaw = self.pose()
        n = self.num_envs
        xy_o = xy + torch.randn_like(xy) * c.pose_noise
        yaw_o = yaw + torch.randn_like(yaw) * c.yaw_noise
        k = (self.wp_idx[:, None] + torch.arange(c.lookahead, device=self.device)[None]).clamp(max=self.wps.shape[1] - 1)
        rel = self.wps[self._rows[:, None], k] - xy_o[:, None]
        cs, sn = torch.cos(yaw_o)[:, None], torch.sin(yaw_o)[:, None]
        local = torch.stack([cs * rel[..., 0] + sn * rel[..., 1], -sn * rel[..., 0] + cs * rel[..., 1]], dim=-1)
        raw = (self.lidar(xy_o, yaw_o) + torch.randn(n, c.lidar_rays, device=self.device) * c.lidar_noise).clamp(0.0, c.lidar_max)
        v_enc, w_enc = self.encoders()
        obs = torch.cat([local.reshape(n, -1) / c.wp_scale, raw / c.lidar_max,
                         torch.stack([v_enc / c.max_lin, w_enc / c.max_ang,
                                      self.true_twist()[1] / c.max_ang], dim=-1),
                         self.action], dim=-1)
        return {"policy": obs}
