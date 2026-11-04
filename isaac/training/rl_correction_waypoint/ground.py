"""이 task의 배경 — 지형(평평 + 요철) · 마찰 패치. env의 조건(웨이포인트 · 보상 · 모터 랜덤)과 따로.

BaseEnv가 장면을 만들 때 ground_cfg()(지형 cfg) · ground_objects()(지형 위 물건)를 부른다 — 이 파일의 두 함수가 그 내용.
같은 배경을 학습 env · run-world 키보드 운전이 쓴다 (calibration 측정은 평면: GroundCfg(uneven=False, friction_patches=0)).
"""
from __future__ import annotations

import math

import torch

import isaaclab.terrains as terrain_gen
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.utils import configclass

from envs.base_env_cfg import GROUND as _GROUND


@configclass
class GroundCfg:
    """배경 설정 — Hydra: env.ground.uneven=false · env.ground.friction_patches=0 …"""

    uneven: bool = True                       # 평평 + 요철 칸 (턱 1~3 cm) · false = 평면만
    # 마찰 패치 — env마다 출발점 둘레에 얇은 판 (노랑 = 끈적, 파랑 = 미끄럼). 바퀴와 평균 결합. 0 = 끔
    friction_patches: int = 8                 # env마다 개수 (반은 끈적 · 반은 미끄럼, 랜덤)
    patch_radius: tuple = (0.3, 3.0)          # [m] 출발점에서 거리 (웨이포인트 경로가 지나가는 범위)
    patch_size: tuple = (0.4, 0.9)            # [m] 한 변
    sticky_friction: tuple = (1.10, 0.90)     # (static, dynamic)
    slippery_friction: tuple = (0.20, 0.15)


PLANE = GroundCfg(uneven=False, friction_patches=0)   # calibration 측정용 (순수 sim)


def make_terrain(g: GroundCfg) -> TerrainImporterCfg:
    """바닥 — Isaac Lab 지형 생성기 (anymal_c_direct Rough와 같은 방식). uneven이 꺼지면 평면.

    8 m 칸을 격자로 깔고 칸마다 종류 랜덤 (curriculum 없음): 평평 · 요철(낮은 턱 1~3 cm). 언덕(경사)은 없다.
    env는 칸 가운데(1 m 평평한 판)에서 출발 — 한 칸에 로봇 여럿이 겹쳐도 서로 충돌 안 함.
    """
    sub = {"flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.2)}
    if g.uneven:
        sub["bumps"] = terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.4, obstacle_height_mode="fixed", obstacle_width_range=(0.30, 0.55),
            obstacle_height_range=(0.01, 0.03), num_obstacles=30, platform_width=1.0)
    if len(sub) == 1:
        return TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane", physics_material=_GROUND)
    gen = TerrainGeneratorCfg(size=(8.0, 8.0), border_width=4.0, num_rows=8, num_cols=8,
                              horizontal_scale=0.05, vertical_scale=0.002, slope_threshold=0.75,
                              color_scheme="height", sub_terrains=sub)
    return TerrainImporterCfg(prim_path="/World/ground", terrain_type="generator", terrain_generator=gen,
                              physics_material=_GROUND, visual_material=None)


def spawn_friction_patches(env, g: "GroundCfg") -> None:
    """마찰 패치 — env마다 출발점 둘레에 노랑(끈적) · 파랑(미끄럼) 얇은 판. 지형 prim 아래라 지형처럼 충돌 (지형 = 충돌 필터의 global).
    판 두께 1 cm, 윗면이 바닥 +1 mm (바퀴가 그대로 지나간다). 배치는 seed 고정 (학습마다 같은 자리)."""
    import random

    import isaaclab.sim as sim_utils
    from isaaclab.utils.math import quat_from_euler_xyz

    c = g
    if c.friction_patches <= 0:
        return
    root = f"{env.cfg.terrain.prim_path}/patches"
    kinds = (("sticky", c.sticky_friction, (1.0, 0.8, 0.1)), ("slippery", c.slippery_friction, (0.2, 0.5, 1.0)))
    for name, fr, _ in kinds:
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=fr[0], dynamic_friction=fr[1], restitution=0.0,
                                             friction_combine_mode="average")
        mat.func(f"/World/Materials/patch_{name}", mat)
    rng = random.Random(env.cfg.seed or 0)
    for i, (ox, oy, oz) in enumerate(env._terrain.env_origins.tolist()):
        for j in range(int(c.friction_patches)):
            name, _, color = kinds[rng.random() < 0.5]
            ang, rad = rng.uniform(0.0, 2.0 * math.pi), rng.uniform(*c.patch_radius)
            cfg = sim_utils.CuboidCfg(size=(rng.uniform(*c.patch_size), rng.uniform(*c.patch_size), 0.01),
                                      collision_props=sim_utils.CollisionPropertiesCfg(),
                                      visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.8))
            q = quat_from_euler_xyz(torch.tensor(0.0), torch.tensor(0.0), torch.tensor(rng.uniform(0.0, math.pi)))
            path = f"{root}/env_{i}_{j}"
            cfg.func(path, cfg, translation=(ox + rad * math.cos(ang), oy + rad * math.sin(ang), oz - 0.004),
                     orientation=tuple(q.tolist()))
            sim_utils.bind_physics_material(f"{path}/geometry/mesh", f"/World/Materials/patch_{name}")
