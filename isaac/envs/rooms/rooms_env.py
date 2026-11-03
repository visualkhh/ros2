"""RoomsEnv — 기본 환경 + 방 여러 개. 방 i 가운데 = (i × room_spacing, 0). `next_room()` = 다음 방으로 순간이동."""
from __future__ import annotations

import math
import random
from collections.abc import Sequence

import isaaclab.sim as sim_utils

from envs.default.default_env import DefaultEnv

FREE_RADIUS = 0.75      # 방 가운데 로봇 자리 — 장애물 금지
WALL_H, WALL_T = 1.1, 0.05
PALETTE = ((0.75, 0.15, 0.12), (0.12, 0.55, 0.20), (0.15, 0.30, 0.75),
           (0.75, 0.55, 0.10), (0.45, 0.20, 0.60), (0.20, 0.60, 0.65))


class RoomsEnv(DefaultEnv):
    def _setup_scene(self):
        super()._setup_scene()
        self.rooms = spawn_rooms(self.cfg)
        self.room = 0

    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        if getattr(self, "rooms", None):
            self.place(self.rooms[self.room]["origin"])

    def next_room(self) -> None:
        if not self.rooms:
            return
        self.room = (self.room + 1) % len(self.rooms)
        rm = self.rooms[self.room]
        self.place(rm["origin"])
        self.look_at(rm["origin"])
        print(f"[rooms] 방 {self.room + 1}/{len(self.rooms)} · {rm['size']:.1f} m · 장애물 {rm['obstacles']}개", flush=True)


def spawn_rooms(cfg) -> list[dict]:
    """cfg.rooms개 방 (벽 4 + 장애물 3~6개, 방마다 크기 랜덤). 방 i 가운데 = (i × room_spacing, 0). 반환: [{origin, size, obstacles}]."""
    if cfg.rooms <= 0:
        return []
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=cfg.obstacle_friction[0],
                                         dynamic_friction=cfg.obstacle_friction[1],
                                         restitution=0.0, friction_combine_mode="average")
    mat.func("/World/Materials/obstacle", mat)
    rooms = []
    for i in range(cfg.rooms):
        rng = random.Random(cfg.room_seed + i * 1000)
        size = random.Random(cfg.room_seed + i * 7919).uniform(*cfg.room_size)
        origin = (i * cfg.room_spacing, 0.0, 0.0)
        half, ox, tag = size / 2.0, origin[0], f"/World/rooms/room_{i}"
        wall = (0.72, 0.72, 0.70)
        for name, dims, pos in (("px", (WALL_T, size, WALL_H), (ox + half, 0.0)), ("nx", (WALL_T, size, WALL_H), (ox - half, 0.0)),
                                ("py", (size, WALL_T, WALL_H), (ox, half)), ("ny", (size, WALL_T, WALL_H), (ox, -half))):
            _box(f"{tag}/wall_{name}", dims, (*pos, WALL_H / 2), wall)
        obstacles = _sample_obstacles(rng, half, rng.randint(3, 6))
        for j, (kind, dims, x, y) in enumerate(obstacles):
            color = PALETTE[(i * 3 + j) % len(PALETTE)]
            path = f"{tag}/obstacle_{j}"
            if kind == "box":
                _box(path, dims, (ox + x, y, dims[2] / 2), color, yaw=rng.uniform(-math.pi, math.pi))
            else:
                c = sim_utils.CylinderCfg(radius=dims[0], height=dims[1], collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                                          visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.6))
                c.func(path, c, translation=(ox + x, y, dims[1] / 2))
            sim_utils.bind_physics_material(f"{path}/geometry/mesh", "/World/Materials/obstacle")
        rooms.append({"origin": origin, "size": size, "obstacles": len(obstacles)})
    return rooms


def _box(path, size, pos, color, yaw: float = 0.0):
    cfg = sim_utils.CuboidCfg(size=size, collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                              visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.6))
    quat = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) if yaw else None
    cfg.func(path, cfg, translation=pos, orientation=quat)


def _sample_obstacles(rng: random.Random, half: float, count: int):
    """겹치지 않게 배치, 로봇 자리(가운데 FREE_RADIUS)는 비움. [(kind, dims, x, y)]."""
    placed, out = [], []
    for _ in range(count):
        for _try in range(200):
            if rng.random() < 0.72:
                w, d, h = rng.uniform(0.20, 0.50), rng.uniform(0.20, 0.50), rng.uniform(0.18, 0.80)
                rad, kind, dims = 0.5 * math.hypot(w, d), "box", (w, d, h)
            else:
                rad, h = rng.uniform(0.10, 0.22), rng.uniform(0.30, 0.80)
                kind, dims = "cyl", (rad, h)
            x, y = rng.uniform(-half + 0.35, half - 0.35), rng.uniform(-half + 0.35, half - 0.35)
            if math.hypot(x, y) < FREE_RADIUS + rad or any(math.hypot(x - px, y - py) < rad + pr + 0.25 for px, py, pr in placed):
                continue
            placed.append((x, y, rad))
            out.append((kind, dims, x, y))
            break
    return out
