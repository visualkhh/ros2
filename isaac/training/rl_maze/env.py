"""RlMazeEnv — 미로 안 랜덤 경로(웨이포인트)를 가장 빨리 따라가는 주행 정책 (nav2 controller 자리).

한 스텝 (20 Hz):
  관측: 다음 웨이포인트 K개(로봇 기준) · 라이다(미로 격자 광선) · 엔코더 v, w · 자이로 w · 이전 행동
  행동 [v, w] ∈ [-1, 1] → × (max_lin, max_ang) → 바퀴 (드라이버 식, BaseEnv)
  보상: 웨이포인트 쪽 진행 + 통과 · 도착 보너스 − 시간 − 충돌 − 떨림 − 후진
  끝: 마지막 웨이포인트 도착 · 벽 충돌(중심이 벽에서 collide_r 안) · 시간
미로는 지형 prim 아래 하나 (모든 env가 같은 미로, env끼리는 충돌 안 함). 라이다 · 충돌은 maze_grid.py 격자로.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.utils.math import euler_xyz_from_quat, quat_from_euler_xyz, sample_uniform

from envs.base_env import ROBOT_Z, BaseEnv
from training.rl_maze.command import RlMazeCommand
from training.rl_maze.maze_grid import MAZE_USD, load
from training.rl_maze.markers import CommandMarkers

if TYPE_CHECKING:
    from training.rl_maze.env_cfg import RlMazeEnvCfg


class RlMazeEnv(BaseEnv):
    cfg: RlMazeEnvCfg

    def __init__(self, cfg: RlMazeEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        c, n, dev = self.cfg, self.num_envs, self.device
        g = load()
        self.occ = torch.as_tensor(g["occ"], device=dev)
        self.wall_dist = torch.as_tensor(g["dist"], dtype=torch.float32, device=dev)
        self.grid_x0, self.grid_y0, self.grid_res = g["origin"][0], g["origin"][1], g["res"]
        self.ray_ang = torch.linspace(0.0, 2.0 * math.pi, c.lidar_rays + 1, device=dev)[:-1]
        self.ray_d = torch.arange(0.0, c.lidar_max, self.grid_res / 2.0, device=dev)
        self.command = RlMazeCommand(dev)
        w = self.command.pts.shape[1]
        self.wps = torch.zeros(n, w, 2, device=dev)
        self.wp_n = torch.ones(n, dtype=torch.long, device=dev)
        self.wp_idx = torch.ones(n, dtype=torch.long, device=dev)      # 지금 향하는 웨이포인트 (0 = 출발점)
        self.prev_dist = torch.zeros(n, device=dev)
        self.progress = torch.zeros(n, device=dev)
        self.action = torch.zeros(n, 2, device=dev)
        self.prev_action = torch.zeros(n, 2, device=dev)
        self.reached = torch.zeros(n, dtype=torch.bool, device=dev)
        self.goal_done = torch.zeros(n, dtype=torch.bool, device=dev)
        self.collided = torch.zeros(n, dtype=torch.bool, device=dev)
        self._rows = torch.arange(n, device=dev)

    # ------------------------------------------------------------------ 배경: 평면 + 미로 (지형 prim 아래 = 모든 env와 충돌)
    def ground_cfg(self):
        return self.cfg.terrain

    def ground_objects(self) -> None:
        import isaaclab.sim as sim_utils

        path = f"{self.cfg.terrain.prim_path}/maze"
        maze = sim_utils.UsdFileCfg(usd_path=MAZE_USD)
        maze.func(path, maze)
        # MaterialX 재질 대신 단색 (RTX가 MaterialX를 컴파일하다 화면이 안 나옴) — 미로 전체에 덮어씀 (충돌 · 물리 재질은 그대로)
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.23, 0.25, 0.34), roughness=0.6)
        look.func("/World/Looks/maze_plain", look)
        sim_utils.bind_visual_material(path, "/World/Looks/maze_plain", stronger_than_descendants=True)

    # ------------------------------------------------------------------ 자세 · 센서
    def pose(self) -> tuple[torch.Tensor, torch.Tensor]:
        """미로 기준 (x, y) [n, 2] · yaw [n] (진짜 값)."""
        d = self.robot.data
        _, _, yaw = euler_xyz_from_quat(d.root_quat_w.torch)
        return d.root_pos_w.torch[:, :2], torch.atan2(torch.sin(yaw), torch.cos(yaw))

    def _cell(self, x: torch.Tensor, y: torch.Tensor):
        ny, nx = self.occ.shape
        ix = ((x - self.grid_x0) / self.grid_res).long().clamp(0, nx - 1)
        iy = ((y - self.grid_y0) / self.grid_res).long().clamp(0, ny - 1)
        return iy, ix

    def lidar(self, xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        """광선마다 첫 벽까지 거리 [n, R] (없으면 lidar_max) — 미로 격자를 반 칸 간격으로 짚어 간다."""
        ang = yaw[:, None] + self.ray_ang[None]                      # [n, R]
        px = xy[:, 0, None, None] + torch.cos(ang)[..., None] * self.ray_d
        py = xy[:, 1, None, None] + torch.sin(ang)[..., None] * self.ray_d
        hit = self.occ[self._cell(px, py)]                           # [n, R, S]
        first = torch.where(hit.any(-1), hit.float().argmax(-1), torch.full_like(hit[..., 0], len(self.ray_d) - 1, dtype=torch.long))
        return torch.where(hit.any(-1), self.ray_d[first], torch.full_like(ang, self.cfg.lidar_max))

    def goal_point(self) -> torch.Tensor:
        return self.wps[self._rows, self.wp_idx.clamp(max=self.wps.shape[1] - 1)]

    # ------------------------------------------------------------------ 한 스텝
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        c = self.cfg
        self.prev_action = self.action
        self.action = actions.clamp(-1.0, 1.0)
        v = (self.action[:, 0] * c.max_lin).clamp(min=c.v_min_ratio * c.max_lin)
        super()._pre_physics_step(torch.stack([v, self.action[:, 1] * c.max_ang], dim=-1))

    def _get_dones(self):
        c = self.cfg
        xy, _ = self.pose()
        d = torch.linalg.norm(self.goal_point() - xy, dim=-1)
        self.reached = d < c.wp_tol
        nxt = (self.wp_idx + 1).clamp(max=self.wps.shape[1] - 1)   # 다음 점이 더 가까우면 건너뛰기 (모서리 질러가기)
        skip = torch.linalg.norm(self.wps[self._rows, nxt] - xy, dim=-1) < d
        adv = (self.reached | skip) & (self.wp_idx < self.wp_n - 1)
        self.goal_done = self.reached & (self.wp_idx >= self.wp_n - 1)
        self.wp_idx = self.wp_idx + adv.long()
        self.progress = self.prev_dist - d                          # 진행 (줄인 거리) — 건너뛴 스텝은 새 목표 기준으로 다시
        self.prev_dist = torch.where(adv, torch.linalg.norm(self.goal_point() - xy, dim=-1), d)
        self.progress = torch.where(adv, torch.zeros_like(d), self.progress)
        self.reached = adv
        self.collided = self.wall_dist[self._cell(xy[:, 0], xy[:, 1])] < c.collide_r
        time_out = self.episode_length_buf >= self.max_episode_length
        return self.goal_done | self.collided, time_out

    def _get_rewards(self) -> torch.Tensor:
        c = self.cfg
        v_enc, _ = self.encoders()
        r = (c.w_progress * self.progress + c.w_reach * self.reached.float() + c.w_goal * self.goal_done.float()
             - c.w_time - c.w_collide * self.collided.float()
             - c.w_smooth * (self.action - self.prev_action).square().sum(-1) - c.w_back * (-v_enc).clamp(min=0.0))
        self.extras["log"] = {"Metrics/waypoints_done": (self.wp_idx - 1).float().mean(),
                              "Metrics/goal_rate": self.goal_done.float().mean(),
                              "Metrics/collision_rate": self.collided.float().mean()}
        return r

    def _get_observations(self) -> dict:
        c = self.cfg
        xy, yaw = self.pose()
        n = self.num_envs
        xy_o = xy + torch.randn_like(xy) * c.pose_noise
        yaw_o = yaw + torch.randn_like(yaw) * c.yaw_noise
        k = (self.wp_idx[:, None] + torch.arange(c.lookahead, device=self.device)[None]).clamp(max=self.wps.shape[1] - 1)
        rel = self.wps[self._rows[:, None], k] - xy_o[:, None]       # [n, K, 2] 미로 기준
        cs, sn = torch.cos(yaw_o)[:, None], torch.sin(yaw_o)[:, None]
        local = torch.stack([cs * rel[..., 0] + sn * rel[..., 1], -sn * rel[..., 0] + cs * rel[..., 1]], dim=-1)
        scan = (self.lidar(xy_o, yaw_o) + torch.randn(n, c.lidar_rays, device=self.device) * c.lidar_noise).clamp(0.0, c.lidar_max)
        v_enc, w_enc = self.encoders()
        obs = torch.cat([local.reshape(n, -1) / c.wp_scale, scan / c.lidar_max,
                         torch.stack([v_enc / c.max_lin, w_enc / c.max_ang, self.true_twist()[1] / c.max_ang], dim=-1),
                         self.action], dim=-1)
        return {"policy": obs}

    # ------------------------------------------------------------------ 리셋: 랜덤 경로의 출발점에서 첫 웨이포인트 쪽으로
    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        ids = self.robot._ALL_INDICES if env_ids is None else torch.as_tensor(env_ids, device=self.device)
        m, dev = len(ids), self.device
        self.wps[ids], self.wp_n[ids] = self.command.sample(m)
        start, first = self.wps[ids, 0], self.wps[ids, 1]
        yaw = torch.atan2(first[:, 1] - start[:, 1], first[:, 0] - start[:, 0]) + sample_uniform(-0.5, 0.5, (m,), dev)
        root = self.robot.data.default_root_state.torch[ids].clone()
        root[:, 0], root[:, 1], root[:, 2] = start[:, 0], start[:, 1], ROBOT_Z
        root[:, 3:7] = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
        self.robot.write_root_pose_to_sim(root[:, :7], ids)
        self.reset_pose[ids] = torch.stack([start[:, 0], start[:, 1], yaw], dim=-1)
        self.wp_idx[ids] = 1
        self.prev_dist[ids] = torch.linalg.norm(first - start, dim=-1)
        self.action[ids] = 0.0
        self.prev_action[ids] = 0.0

    # ------------------------------------------------------------------ 화면 (cfg.debug_vis일 때 BaseEnv가 부른다)
    def debug_vis_setup(self, visible: bool) -> None:
        if visible and getattr(self, "markers", None) is None:
            self.markers = CommandMarkers()
        if getattr(self, "markers", None) is not None:
            self.markers.set_visible(visible)

    def debug_vis_draw(self) -> None:
        self.markers.draw(self, self.vis_path())

    def vis_path(self):
        """노란 점선 = 로봇마다 지금 위치 → 앞 웨이포인트 lookahead개를 잇는 선 (정책이 보는 웨이포인트 그대로)."""
        if not hasattr(self, "wps"):
            return None
        n, k = self.num_envs, self.cfg.lookahead
        idx = (self.wp_idx[:, None] + torch.arange(k, device=self.device)[None]).clamp(max=self.wps.shape[1] - 1)
        ahead = self.wps[self._rows[:, None], idx]                                         # [n, K, 2]
        pts = torch.cat([self.robot.data.root_pos_w.torch[:, None, :2], ahead], dim=1)    # [n, K+1, 2] 로봇부터
        t = torch.linspace(0.0, 1.0, 7, device=self.device)[:-1]                          # 선분마다 6점
        seg = pts[:, :-1, None] + (pts[:, 1:, None] - pts[:, :-1, None]) * t[None, None, :, None]   # [n, K, 6, 2]
        seg = torch.cat([seg.reshape(n, -1, 2), ahead[:, -1:]], dim=1).reshape(-1, 2)
        return torch.cat([seg, torch.full((len(seg), 1), 0.02, device=self.device)], dim=-1)
