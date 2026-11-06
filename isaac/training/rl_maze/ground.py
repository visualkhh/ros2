"""rl_maze 배경 보강 — 마찰 패치.

미로는 ground_objects()가 usda/maze를 깔고, 이 파일은 그 위에 마찰 패치만 얹는다.
rl_maze는 env_spacing=0으로 모든 env가 같은 미로를 공유하므로 패치도 **전역 1세트**만 깐다.
(env별로 256세트를 깔면 같은 좌표에 겹친다.)

패치 = 얇은 판(노랑 = 끈적 · 파랑 = 미끄럼). 지형 prim 아래에 두므로 지형처럼 static collider이고,
바퀴와의 접촉 마찰만 바뀐다. 배치는 seed 고정 — 같은 미로 · 같은 패치에서 학습한다.
"""
from __future__ import annotations

import math
import random

from isaaclab.utils import configclass


PATCH_T, PATCH_TOP = 0.008, 0.002   # [m] 패치 두께 · 윗면 높이 (바닥 z 0 기준 — 2 mm 턱)


@configclass
class GroundCfg:
    """배경 설정 — Hydra: env.ground.friction_patches=0 이면 끔."""

    friction_patches: int = 10                 # 미로 전체에 깔 개수 (전역 1세트)
    patch_size: tuple = (0.35, 0.75)           # [m] 한 변 범위
    sticky_friction: tuple = (1.40, 1.20)      # (static, dynamic) — 바퀴 (1.0, 1.0)와 average
    slippery_friction: tuple = (0.10, 0.06)    # average → 미끄럼이 확실히 느껴지는 값


def spawn_friction_patches(env, g: GroundCfg) -> None:
    """미로의 빈 칸에 끈적(노랑)·미끄럼(파랑) 얇은 판을 전역으로 깐다.

    `env.free_cells`(격자 bool) · `env.wall_dist_np`(벽까지 거리)는 RlMazeEnv가 장면을 만들기 전에
    미로 격자에서 계산해 둔다. 패치는 판의 외접원이 벽에 닿지 않고 서로 겹치지 않게 고른다.
    """
    if g.friction_patches <= 0:
        return
    import isaaclab.sim as sim_utils
    from isaaclab.utils.math import quat_from_euler_xyz

    import torch

    free = torch.nonzero(env.free_cells, as_tuple=False).cpu().numpy()
    if len(free) == 0:
        return
    env.ground_patches = []   # WheelDebug가 바퀴 아래 마찰을 읽는다

    root = f"{env.cfg.terrain.prim_path}/patches"
    kinds = (("sticky", g.sticky_friction, (1.0, 0.8, 0.1)),
             ("slippery", g.slippery_friction, (0.2, 0.5, 1.0)))
    for name, fr, _ in kinds:
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=fr[0], dynamic_friction=fr[1], restitution=0.0,
                                             friction_combine_mode="average")
        mat.func(f"/World/Materials/patch_{name}", mat)

    rng = random.Random(env.cfg.seed or 0)
    placed: list[tuple[float, float, float]] = []
    for j in range(int(g.friction_patches)):
        for _ in range(60):   # 벽 · 다른 패치와 안 겹치는 자리를 찾을 때까지
            size = rng.uniform(*g.patch_size)
            half_diag = 0.5 * size * math.sqrt(2.0)
            iy, ix = (int(v) for v in free[rng.randrange(len(free))])
            if env.wall_dist_np[iy, ix] < half_diag + 0.08:
                continue
            x = env.grid_x0 + (ix + 0.5) * env.grid_res
            y = env.grid_y0 + (iy + 0.5) * env.grid_res
            if any(math.hypot(x - px, y - py) < 0.5 * (size + ps) + 0.06 for px, py, ps in placed):
                continue
            name, fr, color = kinds[rng.random() < 0.5]
            angle = rng.uniform(0.0, math.pi)
            cfg = sim_utils.CuboidCfg(
                size=(size, size, PATCH_T), collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.8))
            q = quat_from_euler_xyz(torch.tensor(0.0), torch.tensor(0.0), torch.tensor(angle))
            path = f"{root}/patch_{j}"
            cfg.func(path, cfg, translation=(x, y, PATCH_TOP - PATCH_T / 2), orientation=tuple(q.tolist()))
            sim_utils.bind_physics_material(f"{path}/geometry/mesh", f"/World/Materials/patch_{name}")
            env.ground_patches.append({"x": x, "y": y, "size": size, "angle": angle, "kind": name,
                                       "static": fr[0], "dynamic": fr[1]})
            placed.append((x, y, size))
            break
