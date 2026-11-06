"""rl_maze 동적 장애물 — 걷는 사람 · 공 (전역 1세트).

미로가 하나(env_spacing=0)라 동적 장애물도 **전역 1세트**다. 위치·속도는 env와 무관하게 움직이고,
모든 env의 라이다·충돌 판정이 같은 장애물을 본다.
  - 사람: 원(collision) + 몸통/머리/팔/다리/손 시각 마커. 걷는 시늉(팔·다리 스윙)을 하고,
    가끔 1~3 s 멈추며, 이따금 로봇 앞쪽 빈 칸에 갑자기 나타난다.
  - 공: 원(collision) + sphere 마커. 벽에 튕기고 느려지면 다시 굴러간다.
물리 rigid body가 아니라 kinematic이다 — 학습·시각화에는 충분하고, 충돌은 env가 원으로 판정한다.
"""
from __future__ import annotations

import math

import torch


class DynamicObstacles:
    def __init__(self, env):
        self.env = env
        c = env.cfg
        self.device = env.device
        self.n_people = int(c.n_people)
        self.n_balls = int(c.n_balls)
        self.person_r = float(c.person_radius)
        self.ball_r = float(c.ball_radius)
        self.people_pos = torch.zeros(self.n_people, 2, device=self.device)
        self.people_vel = torch.zeros(self.n_people, 2, device=self.device)
        self.people_yaw = torch.zeros(self.n_people, device=self.device)
        self.people_phase = torch.zeros(self.n_people, device=self.device)
        self.people_color = torch.zeros(self.n_people, dtype=torch.long, device=self.device)
        self.ball_pos = torch.zeros(self.n_balls, 2, device=self.device)
        self.ball_vel = torch.zeros(self.n_balls, 2, device=self.device)
        self._turn_t = torch.zeros(self.n_people, device=self.device)
        self._pause_t = torch.zeros(self.n_people, device=self.device)
        self._stuck_t = torch.zeros(self.n_people, device=self.device)
        self._block_t = torch.zeros(self.n_people, device=self.device)
        self._last_pos = torch.zeros(self.n_people, 2, device=self.device)
        self._ball_stuck_t = torch.zeros(self.n_balls, device=self.device)
        self._ball_block_t = torch.zeros(self.n_balls, device=self.device)
        self._last_ball = torch.zeros(self.n_balls, 2, device=self.device)
        self._respawn_t = float(c.dynamic_respawn_s)
        self._kick_t = 3.0
        self.markers = None
        self.reset()

    # --------------------------------------------------------------- 위치
    def _random_free(self, k: int) -> torch.Tensor:
        free = getattr(self.env, "free_xy", None)
        if free is None or len(free) == 0:
            free = torch.nonzero(self.env.free_cells, as_tuple=False).float()
            free = torch.stack([self.env.grid_x0 + (free[:, 1] + 0.5) * self.env.grid_res,
                                self.env.grid_y0 + (free[:, 0] + 0.5) * self.env.grid_res], dim=-1)
        i = torch.randint(0, len(free), (k,), device=free.device)
        return free[i]

    def _random_vel(self, k: int, lo: float, hi: float) -> torch.Tensor:
        ang = torch.rand(k, device=self.device) * (2.0 * math.pi)
        spd = lo + torch.rand(k, device=self.device) * (hi - lo)
        return torch.stack([torch.cos(ang) * spd, torch.sin(ang) * spd], dim=-1)

    def _is_free(self, xy: torch.Tensor, radius: float) -> torch.Tensor:
        iy, ix = self.env._cell(xy[:, 0], xy[:, 1])
        return (~self.env.occ[iy, ix]) & (self.env.wall_dist[iy, ix] >= radius + 0.05)

    def _spread_free(self, k: int, avoid: list[torch.Tensor], gap: float) -> torch.Tensor:
        """서로 gap 이상 떨어진 빈 칸 k개 (사람 · 공이 겹쳐 보이지 않게)."""
        fixed = torch.cat(avoid) if avoid else None
        pts: list[torch.Tensor] = []
        for _ in range(400):
            if len(pts) == k:
                break
            p = self._random_free(1)[0]
            ref = torch.cat([fixed, torch.stack(pts)]) if pts and fixed is not None else (
                torch.stack(pts) if pts else fixed)
            if ref is None or torch.linalg.norm(ref - p, dim=-1).min() >= gap:
                pts.append(p)
        while len(pts) < k:   # 자리를 못 찾으면 겹쳐도 채운다
            pts.append(self._random_free(1)[0])
        return torch.stack(pts)

    def reset(self) -> None:
        c = self.env.cfg
        if self.n_people:
            self.people_pos = self._spread_free(self.n_people, [], 2.0 * self.person_r + 0.5)
            self.people_vel = self._random_vel(self.n_people, *c.person_speed)
            self.people_yaw = torch.atan2(self.people_vel[:, 1], self.people_vel[:, 0])
            self.people_phase = torch.rand(self.n_people, device=self.device) * (2.0 * math.pi)
            self.people_color = torch.randint(0, 3, (self.n_people,), device=self.device)
            self._turn_t = 1.0 + torch.rand(self.n_people, device=self.device) * 3.0
            self._pause_t = torch.zeros(self.n_people, device=self.device)
            self._stuck_t = torch.zeros(self.n_people, device=self.device)
            self._block_t = torch.zeros(self.n_people, device=self.device)
            self._last_pos = self.people_pos.clone()
        if self.n_balls:
            self.ball_pos = self._spread_free(self.n_balls, [self.people_pos] if self.n_people else [],
                                               self.person_r + self.ball_r + 0.20)
            self.ball_vel = self._random_vel(self.n_balls, *c.ball_speed)
            self._ball_stuck_t = torch.zeros(self.n_balls, device=self.device)
            self._ball_block_t = torch.zeros(self.n_balls, device=self.device)
            self._last_ball = self.ball_pos.clone()
        self._respawn_t = float(c.dynamic_respawn_s)
        self._kick_t = 3.0

    def _clear_of_dynamics(self, p: torch.Tensor, ignore_person: int) -> bool:
        """p [1,2]가 다른 사람/공과 너무 가깝지 않은가."""
        if self.n_people > 1:
            keep = [i for i in range(self.n_people) if i != ignore_person]
            if torch.linalg.norm(self.people_pos[keep] - p, dim=-1).min() < 2.0 * self.person_r + 0.5:
                return False
        if self.n_balls and torch.linalg.norm(self.ball_pos - p, dim=-1).min() < self.person_r + self.ball_r + 0.20:
            return False
        return True

    def _far_free(self, min_robot: float = 1.2) -> torch.Tensor:
        """로봇들에서 min_robot 이상 떨어진 빈 칸 (교착 방지 텔레포트)."""
        robot_xy = self.env.pose()[0] if self.env.num_envs else None
        for _ in range(50):
            p = self._random_free(1)[0]
            if robot_xy is None or len(robot_xy) == 0 or torch.linalg.norm(robot_xy - p, dim=-1).min() >= min_robot:
                return p
        return self._random_free(1)[0]

    def _teleport_person(self, i: int) -> None:
        """막고 있는 사람을 로봇에서 먼 빈 칸으로 — 사라졌다가 다른 곳에 나타나는 것처럼."""
        c = self.env.cfg
        self.people_pos[i] = self._far_free(1.2)
        self.people_vel[i] = self._random_vel(1, *c.person_speed)[0]
        self.people_yaw[i] = torch.atan2(self.people_vel[i, 1], self.people_vel[i, 0])
        self.people_phase[i] = float(torch.rand((), device=self.device)) * (2.0 * math.pi)
        self.people_color[i] = int(torch.randint(0, 3, (1,), device=self.device))
        self._pause_t[i] = 0.0
        self._turn_t[i] = 2.0
        self._stuck_t[i] = 0.0
        self._block_t[i] = 0.0
        self._last_pos[i] = self.people_pos[i]

    def _teleport_ball(self, i: int) -> None:
        c = self.env.cfg
        self.ball_pos[i] = self._far_free(1.0)
        self.ball_vel[i] = self._random_vel(1, *c.ball_speed)[0]
        self._ball_stuck_t[i] = 0.0
        self._ball_block_t[i] = 0.0
        self._last_ball[i] = self.ball_pos[i]

    def _respawn_person(self) -> None:
        """사람 한 명을 로봇 한 대의 앞쪽 빈 칸에 갑자기 나타나게 한다 (라이다 안 · 충돌 거리 밖)."""
        if not self.n_people:
            return
        c = self.env.cfg
        i = int(torch.randint(0, self.n_people, (1,), device=self.device))
        xy, yaw = self.env.pose()
        if len(xy) == 0:
            self.people_pos[i] = self._random_free(1)[0]
            return
        j = int(torch.randint(0, len(xy), (1,), device=self.device))
        p0 = xy[j]
        for _ in range(16):   # 앞쪽 ±55°에서 고른다
            ang = yaw[j] + float(torch.empty((), device=self.device).uniform_(-0.95, 0.95))
            rad = float(torch.empty((), device=self.device).uniform_(c.dynamic_spawn_min, c.dynamic_spawn_max))
            cand = p0 + torch.tensor([math.cos(float(ang)), math.sin(float(ang))], device=self.device) * rad
            cand = cand[None]
            if not bool(self._is_free(cand, self.person_r)[0]):
                continue
            # 다른 로봇 위에 나타나 즉시 충돌시키지 않는다
            if len(xy) > 1 and torch.linalg.norm(xy - cand, dim=-1).min() < 0.65:
                continue
            if not self._clear_of_dynamics(cand, i):
                continue
            self.people_pos[i] = cand[0]
            self.people_vel[i] = self._random_vel(1, *c.person_speed)[0]
            self.people_yaw[i] = torch.atan2(self.people_vel[i, 1], self.people_vel[i, 0])
            self.people_phase[i] = float(torch.rand((), device=self.device)) * (2.0 * math.pi)
            self.people_color[i] = int(torch.randint(0, 3, (1,), device=self.device))
            self._pause_t[i] = 0.0
            self._turn_t[i] = 2.0
            self._stuck_t[i] = 0.0
            self._block_t[i] = 0.0
            self._last_pos[i] = self.people_pos[i]
            return
        self._teleport_person(i)

    # --------------------------------------------------------------- 한 스텝
    def update(self, dt: float) -> None:
        c = self.env.cfg
        if self.n_people:
            self._turn_t -= dt
            self._pause_t = (self._pause_t - dt).clamp(min=0.0)
            turn = (self._turn_t <= 0.0) & (self._pause_t <= 0.0)
            if turn.any():
                idx = torch.nonzero(turn, as_tuple=False).squeeze(-1)
                k = len(idx)
                pause = torch.rand(k, device=self.device) < c.person_pause_chance
                if pause.any():   # 가끔 잠깐 서 있는다 (계속 돌아다니지 않게)
                    j = idx[pause]
                    self.people_vel[j] = 0.0
                    self._pause_t[j] = c.person_pause_s[0] + torch.rand(len(j), device=self.device) * (
                        c.person_pause_s[1] - c.person_pause_s[0])
                    self._turn_t[j] = 0.0
                move = ~pause
                if move.any():
                    j = idx[move]
                    self.people_vel[j] = self._random_vel(len(j), *c.person_speed)
                    self._turn_t[j] = 1.5 + torch.rand(len(j), device=self.device) * 3.0
            cand = self.people_pos + self.people_vel * dt
            ok = self._is_free(cand, self.person_r)
            self.people_pos = torch.where(ok[:, None], cand, self.people_pos)
            if (~ok).any():
                k = int((~ok).sum())
                self.people_vel[~ok] = self._random_vel(k, *c.person_speed)
            moving = self.people_vel.norm(dim=-1) > 0.05
            yaw = torch.atan2(self.people_vel[:, 1], self.people_vel[:, 0])
            self.people_yaw = torch.where(moving, yaw, self.people_yaw)
            speed = self.people_vel.norm(dim=-1)
            self.people_phase = (self.people_phase + dt * 7.0 * speed) % (2.0 * math.pi)

            # 교착 방지 — 벽에 끼거나 로봇 앞을 오래 막으면 먼 곳으로 사라졌다 다시 나타난다.
            moved = torch.linalg.norm(self.people_pos - self._last_pos, dim=-1)
            stuck = (moved < 0.005) & (self._pause_t <= 0.0)
            self._stuck_t = torch.where(stuck, self._stuck_t + dt, torch.zeros_like(self._stuck_t))
            self._last_pos = self.people_pos.clone()
            if self.env.num_envs <= 4:   # 키보드/소수 env — 로봇 앞을 막고 서 있으면 텔레포트
                rxy = self.env.pose()[0]
                dmin = torch.linalg.norm(self.people_pos[:, None, :] - rxy[None, :, :], dim=-1).min(dim=1).values
                near = dmin < (self.person_r + 0.30)
                stationary = moved < 0.005
                self._block_t = torch.where(near & (stationary | (self._pause_t > 0.0)),
                                            self._block_t + dt, torch.zeros_like(self._block_t))
            else:
                self._block_t = torch.zeros_like(self._block_t)
            need = (self._stuck_t > 2.0) | (self._block_t > 2.0)
            for i in torch.nonzero(need, as_tuple=False).squeeze(-1).tolist():
                self._teleport_person(int(i))

            self._respawn_t -= dt
            if self._respawn_t <= 0.0:
                self._respawn_person()
                self._respawn_t = float(c.dynamic_respawn_s)

        if self.n_balls:
            cand = self.ball_pos + self.ball_vel * dt
            ok = self._is_free(cand, self.ball_r)
            self.ball_vel = torch.where(ok[:, None], self.ball_vel * 0.997, -self.ball_vel * 0.75)
            self.ball_pos = torch.where(ok[:, None], cand, self.ball_pos)

            moved = torch.linalg.norm(self.ball_pos - self._last_ball, dim=-1)
            stuck = moved < 0.002
            self._ball_stuck_t = torch.where(stuck, self._ball_stuck_t + dt, torch.zeros_like(self._ball_stuck_t))
            self._last_ball = self.ball_pos.clone()
            if self.env.num_envs <= 4:
                rxy = self.env.pose()[0]
                dmin = torch.linalg.norm(self.ball_pos[:, None, :] - rxy[None, :, :], dim=-1).min(dim=1).values
                near = dmin < (self.ball_r + 0.30)
                self._ball_block_t = torch.where(near & stuck, self._ball_block_t + dt,
                                                 torch.zeros_like(self._ball_block_t))
            else:
                self._ball_block_t = torch.zeros_like(self._ball_block_t)
            need = (self._ball_stuck_t > 2.0) | (self._ball_block_t > 2.0)
            for i in torch.nonzero(need, as_tuple=False).squeeze(-1).tolist():
                self._teleport_ball(int(i))

            self._kick_t -= dt
            if self._kick_t <= 0.0:
                slow = self.ball_vel.norm(dim=-1) < 0.15
                if slow.any():
                    self.ball_vel[slow] = self._random_vel(int(slow.sum()), *c.ball_speed)
                self._kick_t = 3.0

    # --------------------------------------------------------------- 센서 · 충돌
    def _centers_radii(self):
        centers = torch.cat([self.people_pos, self.ball_pos], dim=0)
        radii = torch.cat([
            torch.full((self.n_people,), self.person_r, device=self.device),
            torch.full((self.n_balls,), self.ball_r, device=self.device),
        ])
        return centers, radii

    def clearance(self, xy: torch.Tensor) -> torch.Tensor:
        """로봇 중심에서 장애물 표면까지 최소거리 [n] (장애물 없으면 큼)."""
        centers, radii = self._centers_radii()
        if len(centers) == 0:
            return torch.full((xy.shape[0],), 1e6, device=self.device)
        d = torch.linalg.norm(xy[:, None, :] - centers[None, :, :], dim=-1) - radii[None, :]
        return d.min(dim=-1).values

    def raycast(self, xy: torch.Tensor, yaw: torch.Tensor, angles: torch.Tensor | None = None) -> torch.Tensor:
        """[n, R] 동적 장애물까지 거리 — env.lidar()가 정적 미로와 min 한다. angles 없으면 정책용 36방향."""
        centers, radii = self._centers_radii()
        a = self.env.ray_ang if angles is None else angles
        n, r = xy.shape[0], a.shape[0]
        if len(centers) == 0:
            return torch.full((n, r), float(self.env.cfg.lidar_max), device=self.device)
        ang = yaw[:, None] + a[None]
        dirs = torch.stack([torch.cos(ang), torch.sin(ang)], dim=-1)      # [n, R, 2]
        rel = centers[None, None, :, :] - xy[:, None, None, :]            # [n, R, M, 2]
        proj = (rel * dirs[:, :, None, :]).sum(-1)                         # [n, R, M]
        perp2 = (rel * rel).sum(-1) - proj * proj
        r2 = radii * radii
        hit = (proj > 0.0) & (perp2 < r2[None, None, :])
        t = proj - torch.sqrt(torch.clamp(r2[None, None, :] - perp2, min=0.0))
        t = torch.where(hit, t, torch.full_like(t, float("inf")))
        return t.min(dim=-1).values.clamp(min=0.0, max=float(self.env.cfg.lidar_max))

    # --------------------------------------------------------------- 화면
    def setup_visuals(self) -> None:
        if self.markers is not None:
            return
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def color(rgb):
            return sim_utils.PreviewSurfaceCfg(diffuse_color=rgb)

        skin, hair, pants = (0.95, 0.74, 0.58), (0.16, 0.10, 0.07), (0.10, 0.12, 0.18)
        shirts = ((0.12, 0.32, 0.82), (0.78, 0.15, 0.15), (0.16, 0.52, 0.24))
        cfg = {}
        for i, rgb in enumerate(shirts):   # 0~2 몸통 색
            cfg[f"body_{i}"] = sim_utils.CapsuleCfg(radius=0.15, height=0.60, visual_material=color(rgb))
        cfg["head"] = sim_utils.SphereCfg(radius=0.125, visual_material=color(skin))     # 3
        cfg["hair"] = sim_utils.SphereCfg(radius=0.128, visual_material=color(hair))     # 4
        cfg["hand"] = sim_utils.SphereCfg(radius=0.045, visual_material=color(skin))     # 5
        cfg["leg"] = sim_utils.CapsuleCfg(radius=0.055, height=0.58, visual_material=color(pants))   # 6
        for i, rgb in enumerate(shirts):   # 7~9 팔 색 (몸통과 같은 색)
            cfg[f"arm_{i}"] = sim_utils.CapsuleCfg(radius=0.045, height=0.50, visual_material=color(rgb))
        cfg["ball"] = sim_utils.SphereCfg(radius=self.ball_r, visual_material=color((0.95, 0.42, 0.08)))   # 10
        self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/Visuals/DynamicObstacles",
                                                                    markers=cfg))

    def set_visible(self, on: bool) -> None:
        if self.markers is not None:
            self.markers.set_visibility(on)

    def draw(self) -> None:
        if self.markers is None:
            return
        from isaaclab.utils.math import quat_from_euler_xyz, quat_mul
        dev = self.device
        trans, quat, idx = [], [], []

        def add(pos, q, kind):
            if len(pos) == 0:
                return
            trans.append(pos)
            quat.append(q)
            idx.append(kind if torch.is_tensor(kind) else
                       torch.full((len(pos),), int(kind), dtype=torch.long, device=dev))

        if self.n_people:
            p, yaw = self.people_pos, self.people_yaw
            fwd = torch.stack([torch.cos(yaw), torch.sin(yaw)], dim=-1)       # 진행 방향
            side = torch.stack([-torch.sin(yaw), torch.cos(yaw)], dim=-1)     # 왼쪽
            z = lambda v: torch.full((self.n_people, 1), v, device=dev)  # noqa: E731
            zero = torch.zeros(self.n_people, device=dev)
            q_yaw = quat_from_euler_xyz(zero, zero, yaw)

            def limb(offset, length, tilt):
                """관절(offset)에서 길이 length 다리를 tilt만큼 앞뒤로 — 중심 위치 · 회전."""
                ctr = offset.clone()
                ctr[:, :2] -= fwd * (0.5 * length * torch.sin(tilt))[:, None]
                ctr[:, 2] -= 0.5 * length * torch.cos(tilt)
                return ctr, quat_mul(q_yaw, quat_from_euler_xyz(zero, tilt, zero))

            add(torch.cat([p, z(0.86)], dim=-1), q_yaw, self.people_color)    # 몸통 (셔츠 색)
            add(torch.cat([p, z(1.28)], dim=-1), q_yaw, 3)                    # 머리
            add(torch.cat([p - fwd * 0.018, z(1.315)], dim=-1), q_yaw, 4)     # 머리카락 (뒤·위)
            speed = self.people_vel.norm(dim=-1)
            swing = 0.42 * torch.sin(self.people_phase) * (speed / 0.8).clamp(0.0, 1.25)
            for kind, lateral, length, shoulder in ((6, 0.095, 0.58, 0.56), (7, 0.205, 0.50, 1.04)):
                for sign in (1.0, -1.0):
                    if kind == 6:   # 다리: 양쪽이 반대로
                        tilt = sign * swing
                        hip = torch.cat([p + side * (sign * lateral), z(shoulder)], dim=-1)
                        ctr, q = limb(hip, length, tilt)
                    else:           # 팔: 다리와 반대로 (몸통과 같은 셔츠 색)
                        tilt = -sign * 0.8 * swing
                        hip = torch.cat([p + side * (sign * lateral), z(shoulder)], dim=-1)
                        ctr, q = limb(hip, length, tilt)
                        hand = hip.clone()
                        hand[:, :2] -= fwd * (length * torch.sin(tilt))[:, None]
                        hand[:, 2] = shoulder - length * torch.cos(tilt)
                        add(hand, q_yaw, 5)
                    add(ctr, q, kind if kind == 6 else 7 + self.people_color)

        if self.n_balls:
            p = self.ball_pos
            add(torch.cat([p, torch.full((self.n_balls, 1), self.ball_r, device=dev)], dim=-1),
                quat_from_euler_xyz(torch.zeros(self.n_balls, device=dev), torch.zeros(self.n_balls, device=dev),
                                    torch.zeros(self.n_balls, device=dev)), 10)

        if not trans:
            return
        self.markers.visualize(translations=torch.cat(trans), orientations=torch.cat(quat),
                               scales=torch.ones(sum(len(x) for x in trans), 3, device=dev),
                               marker_indices=torch.cat(idx))
