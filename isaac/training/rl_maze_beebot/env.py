"""RlMazeBeebotEnv — beebot 미로 주행. rl_maze env를 그대로 쓰고 센서만 nvblox height slice에 맞춘다.

  beebot `rl_nav_controller.cpp`의 points 경로 (배포 설정):
    /nvblox_node/combined_esdf_pointcloud (static+dynamic 2D ESDF, z 0.03~0.30 m, 0.05 m voxel)
    → base 변환 → 360방향 빈의 최소거리.
  여기서는 미로 격자 raycast를 같은 360방향에 대해 수행한다. 0.05 m voxel·3.5 m에서
  1°(360방향)가 의미 있는 최대 해상도라 360으로 쓴다 (720은 voxel보다 촘촘해 이득 없음).
  관측은 375차원 — inflation 입력 없이 raw 스캔 (beebot plugin과 같은 계약).
"""
from __future__ import annotations

import torch

from training.rl_maze.env import RlMazeEnv
from training.rl_maze_beebot.env_cfg import RlMazeBeebotEnvCfg


class RlMazeBeebotEnv(RlMazeEnv):
    cfg: RlMazeBeebotEnvCfg

    def toggle_lidar_vis(self, on: bool | None = None) -> bool:
        """L — nvblox height slice point cloud(360°·z=0.3)를 켜고 끈다."""
        if getattr(self, "lidar_markers", None) is None:
            if not self.cfg.debug_vis:
                return False
            from training.rl_maze.markers import LidarMarkers
            self.lidar_markers = LidarMarkers("/Visuals/BeebotPointCloud")
        self.lidar_vis = (not self.lidar_vis) if on is None else bool(on)
        self.lidar_markers.set_visible(self.lidar_vis)
        print(f"[vis] nvblox height slice {'ON' if self.lidar_vis else 'OFF'} · "
              f"360° · {self.cfg.lidar_vis_rays} rays · z=0.10 m "
              f"(raw=청록 · 3.5m=회색)", flush=True)
        return self.lidar_vis

    def lidar_points(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """nvblox 2D ESDF slice와 같은 360° height point cloud (z=0.10 m)."""
        xy, yaw = self.pose()
        n = self.num_envs
        raw = self.lidar(xy, yaw, self.vis_ang)                # [n, R] 360°
        ang = yaw[:, None] + self.vis_ang[None]
        z = torch.full_like(raw, 0.10)
        pts = torch.stack([xy[:, 0, None] + raw * torch.cos(ang),
                           xy[:, 1, None] + raw * torch.sin(ang), z], dim=-1)
        infl = torch.zeros_like(raw)                           # raw만 — magenta inflation 없음
        return pts, pts, raw, infl

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
        scan = (raw - self.inflation[:, None]).clamp(0.0, c.lidar_max)  # 실기 lidar_inflation과 같은 식
        v_enc, w_enc = self.encoders()
        obs = torch.cat([local.reshape(n, -1) / c.wp_scale, scan / c.lidar_max,
                         torch.stack([v_enc / c.max_lin, w_enc / c.max_ang,
                                      self.true_twist()[1] / c.max_ang], dim=-1),
                         self.action], dim=-1)
        return {"policy": obs}
