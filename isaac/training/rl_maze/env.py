"""RlMazeEnv — 미로 안 랜덤 경로(웨이포인트)를 가장 빨리 따라가는 주행 정책 (nav2 controller 자리).

한 스텝 (20 Hz):
  관측: 다음 웨이포인트 K개(로봇 기준) · 라이다(미로 격자 + 동적 장애물, 인플레이션 적용) · 엔코더 v, w · 자이로 w · 이전 행동 · inflation
  행동 [v, w] ∈ [-1, 1] → × (max_lin, max_ang) → 바퀴 (드라이버 식, BaseEnv)
  보상: 안전 최우선 — 웨이포인트 진행 + 도착 보너스 − 시간 − 충돌 − 벽/사람/공 근접 − 떨림 − 후진
  끝: 마지막 웨이포인트 도착 · 벽/사람/공 충돌(중심이 표면에서 collide_r 안) · 시간
미로는 지형 prim 아래 하나 (모든 env가 같은 미로, env끼리는 충돌 안 함). 라이다 · 충돌은 maze_grid.py 격자로.
동적 장애물(사람 · 공)과 마찰 패치도 미로와 같이 전역 1세트 — 모든 env가 같은 것을 본다.
"""
from __future__ import annotations

import math
import os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np
import torch

from isaaclab.utils.math import euler_xyz_from_quat, quat_from_euler_xyz, sample_uniform

from envs.base_env import ROBOT_Z, BaseEnv
from envs.base_env_cfg import GROUND_FRICTION
from training.rl_maze.command import RlMazeCommand
from training.rl_maze.dynamic import DynamicObstacles
from training.rl_maze.ground import spawn_friction_patches
from training.rl_maze.maze_grid import inside, load
from training.rl_maze.markers import CommandMarkers, LidarMarkers, WheelMarkers

if TYPE_CHECKING:
    from training.rl_maze.env_cfg import RlMazeEnvCfg


class RlMazeEnv(BaseEnv):
    cfg: RlMazeEnvCfg
    direct_drive = True   # --command keyboard/square를 CalibrationEnv가 아니라 이 env에서 직접 [v,w]로

    def __init__(self, cfg: RlMazeEnvCfg, render_mode: str | None = None, **kwargs):
        c = cfg
        g = load(os.path.join(c.maze_dir, "grid.npz"))
        self.maze_usd = os.path.join(c.maze_dir, "maze.usda")
        # 배경(ground_objects)이 super().__init__ 안에서 불리므로 격자 · 빈 칸을 먼저 둔다 (아직 CPU).
        self.occ_np = g["occ"]
        self.wall_dist_np = g["dist"].astype(np.float32)
        self.grid_x0, self.grid_y0, self.grid_res = float(g["origin"][0]), float(g["origin"][1]), float(g["res"])
        safe_r = max(float(c.person_radius), float(c.ball_radius), 0.20)
        self.free_cells = torch.as_tensor((self.wall_dist_np >= safe_r + 0.05) & inside(self.occ_np), dtype=torch.bool)
        self.ground_patches = []   # spawn_friction_patches가 채운다 (바퀴 마찰 디버그)
        self.wheel_markers = None
        super().__init__(cfg, render_mode, **kwargs)

        n, dev = self.num_envs, self.device
        self.occ = torch.as_tensor(self.occ_np, device=dev)
        self.wall_dist = torch.as_tensor(self.wall_dist_np, dtype=torch.float32, device=dev)
        self.free_cells = self.free_cells.to(dev)
        cells = torch.nonzero(self.free_cells, as_tuple=False).float()
        self.free_xy = torch.stack([self.grid_x0 + (cells[:, 1] + 0.5) * self.grid_res,
                                    self.grid_y0 + (cells[:, 0] + 0.5) * self.grid_res], dim=-1)
        self.ray_ang = torch.linspace(0.0, 2.0 * math.pi, c.lidar_rays + 1, device=dev)[:-1]
        self.vis_ang = torch.linspace(0.0, 2.0 * math.pi, c.lidar_vis_rays + 1, device=dev)[:-1]
        self.ray_d = torch.arange(0.0, c.lidar_max, self.grid_res / 2.0, device=dev)
        self.command = RlMazeCommand(min_clear=max(c.inflation_range) + c.path_margin,
                                     start_clear=max(0.32, max(c.inflation_range) + c.path_margin))
        w = self.command.pts.shape[1]
        self.wps = torch.zeros(n, w, 2, device=dev)
        self.wp_n = torch.ones(n, dtype=torch.long, device=dev)
        self.wp_idx = torch.ones(n, dtype=torch.long, device=dev)      # 지금 향하는 웨이포인트 (0 = 출발점)
        self.prev_dist = torch.zeros(n, device=dev)
        self.progress = torch.zeros(n, device=dev)
        self.action = torch.zeros(n, 2, device=dev)
        self.prev_action = torch.zeros(n, 2, device=dev)
        self.inflation = torch.full((n,), float(sum(c.inflation_range) / 2.0), device=dev)   # 사용자 입력 (m)
        self.reached = torch.zeros(n, dtype=torch.bool, device=dev)
        self.goal_done = torch.zeros(n, dtype=torch.bool, device=dev)
        self.collided = torch.zeros(n, dtype=torch.bool, device=dev)
        self.wall_hit = torch.zeros(n, dtype=torch.bool, device=dev)
        self.dyn_hit = torch.zeros(n, dtype=torch.bool, device=dev)
        self.grace_steps = torch.full((n,), int(c.spawn_grace_steps), dtype=torch.long, device=dev)
        self.wall_clearance = torch.full((n,), 1e6, device=dev)
        self.dyn_clearance = torch.full((n,), 1e6, device=dev)
        self._rows = torch.arange(n, device=dev)
        self._wheel_bodies, _ = self.robot.find_bodies(["wheel_left_link", "wheel_right_link"], preserve_order=True)
        self._base_dyn_friction = (float(c.wheel_friction[1]) + float(GROUND_FRICTION[1])) / 2.0
        self.dynamic = DynamicObstacles(self)
        self.lidar_markers = None
        self.lidar_vis = False
        self._render_vis = bool(self.cfg.sim.visualizer_cfgs) or bool(getattr(self.sim, "has_gui", False))
        if self._render_vis:
            self.dynamic.setup_visuals()   # 사람/공은 debug_vis와 무관하게 항상 보이게 (바닥 패치처럼)
        if c.debug_vis:
            self.wheel_markers = WheelMarkers()
            if c.lidar_vis:
                self.toggle_lidar_vis(True)
        print(f"[rl_maze] 빈 칸 {len(self.free_xy)} · 사람 {c.n_people} · 공 {c.n_balls} · "
              f"마찰 패치 {c.ground.friction_patches} · 라이다 {c.lidar_rays}방향 · "
              f"경로 {len(self.command.lens)} · inflation {c.inflation_range[0]:.2f}~{c.inflation_range[1]:.2f} m", flush=True)


    # ------------------------------------------------------------------ 배경: 평면 + 미로 (지형 prim 아래 = 모든 env와 충돌)
    def ground_cfg(self):
        return self.cfg.terrain

    def ground_objects(self) -> None:
        import isaaclab.sim as sim_utils

        path = f"{self.cfg.terrain.prim_path}/maze"
        maze = sim_utils.UsdFileCfg(usd_path=self.maze_usd)
        maze.func(path, maze)
        # MaterialX 재질 대신 단색 (RTX가 MaterialX를 컴파일하다 화면이 안 나옴) — 미로 전체에 덮어씀 (충돌 · 물리 재질은 그대로)
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.23, 0.25, 0.34), roughness=0.6)
        look.func("/World/Looks/maze_plain", look)
        sim_utils.bind_visual_material(path, "/World/Looks/maze_plain", stronger_than_descendants=True)
        spawn_friction_patches(self, self.cfg.ground)

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

    def lidar(self, xy: torch.Tensor, yaw: torch.Tensor, angles: torch.Tensor | None = None) -> torch.Tensor:
        """광선마다 첫 벽/동적 장애물까지 거리 [n, R] (없으면 lidar_max). angles 없으면 정책용 36방향."""
        a = self.ray_ang if angles is None else angles
        ang = yaw[:, None] + a[None]                       # [n, R]
        px = xy[:, 0, None, None] + torch.cos(ang)[..., None] * self.ray_d
        py = xy[:, 1, None, None] + torch.sin(ang)[..., None] * self.ray_d
        hit = self.occ[self._cell(px, py)]                           # [n, R, S]
        first = torch.where(hit.any(-1), hit.float().argmax(-1), torch.full_like(hit[..., 0], len(self.ray_d) - 1, dtype=torch.long))
        wall = torch.where(hit.any(-1), self.ray_d[first], torch.full_like(ang, self.cfg.lidar_max))
        dyn = getattr(self, "dynamic", None)
        return wall if dyn is None else torch.minimum(wall, dyn.raycast(xy, yaw, angles))

    def goal_point(self) -> torch.Tensor:
        return self.wps[self._rows, self.wp_idx.clamp(max=self.wps.shape[1] - 1)]

    # ------------------------------------------------------------------ 한 스텝
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        c = self.cfg
        self.grace_steps = (self.grace_steps - 1).clamp(min=0)
        self.dynamic.update(self.policy_dt)
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

        self.wall_clearance = self.wall_dist[self._cell(xy[:, 0], xy[:, 1])]
        self.dyn_clearance = self.dynamic.clearance(xy)
        grace = self.grace_steps > 0
        self.wall_hit = (self.wall_clearance < c.collide_r) & ~grace
        self.dyn_hit = (self.dyn_clearance < c.collide_r) & ~grace
        self.collided = self.wall_hit | self.dyn_hit
        time_out = self.episode_length_buf >= self.max_episode_length
        return self.goal_done | self.collided, time_out

    def _get_rewards(self) -> torch.Tensor:
        c = self.cfg
        v_enc, _ = self.encoders()
        active = (self.grace_steps <= 0).float()
        span = (self.inflation - c.collide_r).clamp(min=1e-3)
        wall_overlap = (self.inflation - self.wall_clearance).clamp(min=0.0)
        dyn_overlap = (self.inflation - self.dyn_clearance).clamp(min=0.0)
        wall_near = (wall_overlap / span).clamp(0.0, 1.0) * active
        dyn_near = (dyn_overlap / span).clamp(0.0, 1.0) * active
        terms = {
            "progress": c.w_progress * self.progress.clamp(-0.25, 0.25),
            "reach": c.w_reach * self.reached.float(),
            "goal": c.w_goal * self.goal_done.float(),
            "time": torch.full_like(self.progress, -c.w_time),
            "collision": -c.w_collide * self.collided.float(),
            "wall": -c.w_wall * wall_near,
            "dynamic": -c.w_dynamic * dyn_near,
            "smooth": -c.w_smooth * (self.action - self.prev_action).square().sum(-1),
            "back": -c.w_back * (-v_enc).clamp(min=0.0),
        }
        r = torch.stack(list(terms.values()), dim=0).sum(dim=0)

        log = self.extras.setdefault("log", {})
        for name, value in terms.items():
            log[f"Reward/{name}"] = value.mean().item()
        log["Reward/total"] = r.mean().item()
        log["Metrics/waypoints_done"] = (self.wp_idx - 1).float().mean().item()
        log["Metrics/goal_rate"] = self.goal_done.float().mean().item()
        log["Metrics/collision_rate"] = self.collided.float().mean().item()
        log["Metrics/wall_collision_rate"] = self.wall_hit.float().mean().item()
        log["Metrics/dyn_collision_rate"] = self.dyn_hit.float().mean().item()
        log["Metrics/wall_clear"] = self.wall_clearance.mean().item()
        log["Metrics/dyn_clear"] = self.dyn_clearance.clamp(max=2.0).mean().item()
        log["Metrics/wall_overlap"] = wall_overlap.mean().item()
        log["Metrics/dyn_overlap"] = dyn_overlap.mean().item()
        log["Metrics/inflation"] = self.inflation.mean().item()
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
        raw = (self.lidar(xy_o, yaw_o) + torch.randn(n, c.lidar_rays, device=self.device) * c.lidar_noise).clamp(0.0, c.lidar_max)
        scan = (raw - self.inflation[:, None]).clamp(0.0, c.lidar_max)   # 정책이 보는 안전거리 (실기 scan_rays와 같은 식)
        v_enc, w_enc = self.encoders()
        obs = torch.cat([local.reshape(n, -1) / c.wp_scale, scan / c.lidar_max,
                         torch.stack([v_enc / c.max_lin, w_enc / c.max_ang, self.true_twist()[1] / c.max_ang], dim=-1),
                         self.action, self.inflation[:, None] / c.inflation_scale], dim=-1)
        return {"policy": obs}

    # ------------------------------------------------------------------ 리셋: 랜덤 경로의 출발점에서 첫 웨이포인트 쪽으로
    def step(self, action):
        out = super().step(action)
        if getattr(self, "dynamic", None) is not None:
            self.dynamic.draw()   # 사람/공은 debug_vis 없이도 항상 갱신
        return out

    def reset(self, **kwargs):
        """전체 리셋 — 동적 장애물도 새로 배치 (학습 중 env별 리셋은 _reset_idx만 부른다)."""
        if hasattr(self, "dynamic"):
            self.dynamic.reset()
        return super().reset(**kwargs)

    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        ids = self.robot._ALL_INDICES if env_ids is None else torch.as_tensor(env_ids, device=self.device)
        m, dev = len(ids), self.device
        self.wps[ids], self.wp_n[ids] = self.command.sample(m, self.device)
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
        self.progress[ids] = 0.0
        self.reached[ids] = False
        self.goal_done[ids] = False
        self.collided[ids] = False
        self.wall_hit[ids] = False
        self.dyn_hit[ids] = False
        self.grace_steps[ids] = int(self.cfg.spawn_grace_steps)
        self.wall_clearance[ids] = 1e6
        self.dyn_clearance[ids] = 1e6
        self.inflation[ids] = sample_uniform(*self.cfg.inflation_range, (m,), dev)

    # ------------------------------------------------------------------ 화면 (cfg.debug_vis일 때 BaseEnv가 부른다)
    def debug_vis_setup(self, visible: bool) -> None:
        if visible and getattr(self, "markers", None) is None:
            self.markers = CommandMarkers()
        if getattr(self, "markers", None) is not None:
            self.markers.set_visible(visible)
        dyn = getattr(self, "dynamic", None)
        if dyn is not None and visible:
            dyn.setup_visuals()   # 사람/공은 debug_vis를 꺼도 계속 보인다
        wm = getattr(self, "wheel_markers", None)
        if visible and wm is None and hasattr(self, "_wheel_bodies"):
            self.wheel_markers = wm = WheelMarkers()
        if wm is not None:
            wm.set_visible(visible)
        if getattr(self, "lidar_markers", None) is not None:
            self.lidar_markers.set_visible(visible and self.lidar_vis)

    def toggle_lidar_vis(self, on: bool | None = None) -> bool:
        """L — 지금 라이다가 재는 점군을 화면에 켜고 끈다 (정적 미로 + 사람/공)."""
        if getattr(self, "lidar_markers", None) is None:
            if not self.cfg.debug_vis:
                return False
            self.lidar_markers = LidarMarkers()
        self.lidar_vis = (not self.lidar_vis) if on is None else bool(on)
        self.lidar_markers.set_visible(self.lidar_vis)
        print(f"[vis] LiDAR point cloud {'ON' if self.lidar_vis else 'OFF'} · 시각화 {self.cfg.lidar_vis_rays}방향 "
              f"(정책 {self.cfg.lidar_rays}) (raw=청록 · inflation=마젠타 · 3.5m=회색 · 마진 안=생략) · inflation "
              f"{self.cfg.inflation_range[0]:.2f}~{self.cfg.inflation_range[1]:.2f} m "
              f"(현재 {float(self.inflation.mean()):.2f})", flush=True)
        return self.lidar_vis

    def lidar_points(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """라이다 점군 — raw [n,R,3] · inflation [n,R,3] · raw 거리 [n,R] · inflation 거리 [n,R]."""
        xy, yaw = self.pose()
        raw = self.lidar(xy, yaw, self.vis_ang)
        infl = (raw - self.inflation[:, None]).clamp(0.0, self.cfg.lidar_max)
        ang = yaw[:, None] + self.vis_ang[None]
        cos, sin = torch.cos(ang), torch.sin(ang)
        z = torch.full_like(raw, LidarMarkers.Z)
        raw_pts = torch.stack([xy[:, 0, None] + raw * cos,
                               xy[:, 1, None] + raw * sin, z], dim=-1)
        infl_pts = torch.stack([xy[:, 0, None] + infl * cos,
                                xy[:, 1, None] + infl * sin, z], dim=-1)
        return raw_pts, infl_pts, raw, infl

    def debug_vis_draw(self) -> None:
        if getattr(self, "markers", None) is not None:
            dots, wps = self.vis_path() or (None, None)
            self.markers.draw(self, dots, wps)
        if getattr(self, "wheel_markers", None) is not None:
            self.wheel_markers.draw(self)
        if getattr(self, "lidar_vis", False) and getattr(self, "lidar_markers", None) is not None:
            raw_pts, infl_pts, raw, infl = self.lidar_points()
            self.lidar_markers.draw(raw_pts, infl_pts, raw, infl, self.cfg.lidar_max)

    def wheel_state(self):
        """(바퀴 월드 위치 [n,2,3], 바퀴 관절 rad/s [n,2], 유효 동마찰 [n,2]) — 디버그 두 줄."""
        d = self.robot.data
        pos = d.body_pos_w.torch[:, self._wheel_bodies]
        jv = torch.nan_to_num(d.joint_vel.torch[:, self._wheels])
        if not self.ground_patches:
            return pos, jv, torch.full_like(jv, self._base_dyn_friction)
        xy = pos[..., :2].detach().cpu().numpy()
        wheel_dyn = float(self.cfg.wheel_friction[1])
        vals = [[self._base_dyn_friction, self._base_dyn_friction] for _ in range(self.num_envs)]
        for i in range(self.num_envs):
            for w in range(2):
                wx, wy = float(xy[i, w, 0]), float(xy[i, w, 1])
                for p in self.ground_patches:
                    dx, dy = wx - p["x"], wy - p["y"]
                    ca, sa = math.cos(p["angle"]), math.sin(p["angle"])
                    lx, ly = ca * dx + sa * dy, -sa * dx + ca * dy
                    if abs(lx) <= p["size"] / 2.0 and abs(ly) <= p["size"] / 2.0:
                        vals[i][w] = (wheel_dyn + float(p["dynamic"])) / 2.0
                        break
        return pos, jv, torch.tensor(vals, device=self.device, dtype=jv.dtype)

    def vis_path(self):
        """(노란 점선, 웨이포인트) — 점선 = 로봇마다 지금 위치 → 앞 웨이포인트 lookahead개를 잇는 그림 ·
        웨이포인트 = 정책이 관측으로 받는 그 점들 (빨간 큰 공)."""
        if not hasattr(self, "wps"):
            return None
        n, k = self.num_envs, self.cfg.lookahead
        idx = (self.wp_idx[:, None] + torch.arange(k, device=self.device)[None]).clamp(max=self.wps.shape[1] - 1)
        ahead = self.wps[self._rows[:, None], idx]                                         # [n, K, 2]
        pts = torch.cat([self.robot.data.root_pos_w.torch[:, None, :2], ahead], dim=1)    # [n, K+1, 2] 로봇부터
        t = torch.linspace(0.0, 1.0, 7, device=self.device)[:-1]                          # 선분마다 6점
        seg = pts[:, :-1, None] + (pts[:, 1:, None] - pts[:, :-1, None]) * t[None, None, :, None]   # [n, K, 6, 2]
        seg = torch.cat([seg.reshape(n, -1, 2), ahead[:, -1:]], dim=1).reshape(-1, 2)
        z = lambda p: torch.cat([p, torch.full((len(p), 1), 0.02, device=self.device)], dim=-1)  # noqa: E731
        return z(seg), z(ahead.reshape(-1, 2))
