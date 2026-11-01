"""wheel_drift task 전용 장면. 한 파일 한 클래스.

벡터화 RL env(수백~수천 개 복제)용이라 calibration의 `Scene`을 상속하지 않는다.
라이프사이클이 다르다: 방/스폰/place가 아니라 ground+마찰 패치+복제다.
task에 맞는 ground·마찰이 바뀌면 여기만 고치면 된다.
"""
from __future__ import annotations

import math
import random

import isaaclab.sim as sim_utils
from isaaclab import cloner
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane


class TaskScene:
    """ground+마찰 패치+바퀴/캐스터 바인딩. env가 `_setup_scene`에서 위임한다."""

    def __init__(self, scene, cfg, device):
        self.scene = scene
        self.cfg = cfg
        self.device = device

    def setup_ground(self):
        c = self.cfg
        slab_world = getattr(c, "randomize_friction", False) or getattr(c, "randomize_uneven", False)
        if slab_world:
            # env마다 ground slab을 따로 만든다. 위에는 마찰 패치 / 요철 패치를 뿌린다.
            spacing = self.scene.cfg.env_spacing
            size = max(1.0, spacing - 0.1)
            pos = cloner.grid_transforms(self.scene.num_envs, spacing, device=self.device)[0]

            if getattr(c, "randomize_friction", False):
                slip_mat = sim_utils.RigidBodyMaterialCfg(
                    static_friction=c.slippery_friction[0], dynamic_friction=c.slippery_friction[1],
                    restitution=0.0, friction_combine_mode="average",
                )
                slip_mat.func("/World/Materials/friction_slippery", slip_mat)
                stick_mat = sim_utils.RigidBodyMaterialCfg(
                    static_friction=c.sticky_friction[0], dynamic_friction=c.sticky_friction[1],
                    restitution=0.0, friction_combine_mode="average",
                )
                stick_mat.func("/World/Materials/friction_sticky", stick_mat)
            if getattr(c, "randomize_uneven", False):
                uneven_mat = sim_utils.RigidBodyMaterialCfg(
                    static_friction=c.ground_friction[0], dynamic_friction=c.ground_friction[1],
                    restitution=0.0, friction_combine_mode="average",
                )
                uneven_mat.func("/World/Materials/uneven", uneven_mat)

            for i, (x, y, _z) in enumerate(pos.detach().cpu().tolist()):
                if getattr(c, "randomize_friction", False):
                    static = random.uniform(*c.ground_static_range)
                    dynamic = random.uniform(*c.ground_dynamic_range)
                else:
                    static, dynamic = c.ground_friction
                mat = sim_utils.RigidBodyMaterialCfg(
                    static_friction=static, dynamic_friction=dynamic,
                    restitution=0.0, friction_combine_mode="average",
                )
                cfg = sim_utils.CuboidCfg(
                    size=(size, size, 0.1),
                    collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                    physics_material=mat,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.45, 0.45)),
                )
                cfg.func(f"/World/grounds/ground_{i}", cfg, translation=(x, y, -0.05))

                # 파랑=미끄럼, 주황=끈적. 얇은 판이라 바퀴가 그대로 지나간다.
                patch_n = int(c.friction_patch_count) if getattr(c, "randomize_friction", False) else 0
                for j in range(patch_n):
                    angle = (2.0 * math.pi * j / max(1, patch_n)
                             + random.uniform(-0.35, 0.35))
                    radius = random.uniform(*c.friction_patch_radius)
                    pw = random.uniform(*c.friction_patch_size)
                    pd = random.uniform(*c.friction_patch_size)
                    sticky = random.random() < 0.5
                    mat_path = ("/World/Materials/friction_sticky" if sticky
                                else "/World/Materials/friction_slippery")
                    color = (1.0, 0.45, 0.10) if sticky else (0.20, 0.50, 1.0)
                    patch = sim_utils.CuboidCfg(
                        size=(pw, pd, 0.01),
                        collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=color, roughness=0.8),
                    )
                    path = f"/World/patches/patch_{i}_{j}"
                    patch.func(path, patch,
                               translation=(x + radius * math.cos(angle),
                                            y + radius * math.sin(angle), -0.004))
                    sim_utils.bind_physics_material(f"{path}/geometry/mesh", mat_path)

                # 요철: 바퀴가 올라타는 낮은 턱. 기하학적 외란 (마찰과 독립).
                if getattr(c, "randomize_uneven", False):
                    half = size / 2.0
                    for j in range(int(c.uneven_patch_count)):
                        bw = random.uniform(*c.uneven_patch_size)
                        bd = random.uniform(*c.uneven_patch_size)
                        bh = random.uniform(*c.uneven_patch_height)
                        bx = x + random.uniform(-half + 0.35, half - 0.35)
                        by = y + random.uniform(-half + 0.35, half - 0.35)
                        bump = sim_utils.CuboidCfg(
                            size=(bw, bd, bh),
                            collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                            visual_material=sim_utils.PreviewSurfaceCfg(
                                diffuse_color=(0.55, 0.55, 0.58), roughness=0.9),
                        )
                        bump_path = f"/World/uneven/bump_{i}_{j}"
                        bump.func(bump_path, bump, translation=(bx, by, bh / 2.0))
                        sim_utils.bind_physics_material(f"{bump_path}/geometry/mesh",
                                                        "/World/Materials/uneven")
        else:
            g = c.ground_friction
            mat = sim_utils.RigidBodyMaterialCfg(
                static_friction=g[0], dynamic_friction=g[1],
                restitution=0.0, friction_combine_mode="average",
            )
            spawn_ground_plane(prim_path="/World/ground",
                               cfg=GroundPlaneCfg(physics_material=mat))

    def bind_robot_materials(self):
        from pxr import UsdPhysics

        c = self.cfg
        wheel_mat = sim_utils.RigidBodyMaterialCfg(
            static_friction=c.wheel_friction[0], dynamic_friction=c.wheel_friction[1],
            restitution=0.0, friction_combine_mode="average",
        )
        wheel_mat.func("/World/Materials/wheel", wheel_mat)
        caster_mat = sim_utils.RigidBodyMaterialCfg(
            static_friction=c.caster_friction[0], dynamic_friction=c.caster_friction[1],
            restitution=0.0, friction_combine_mode="min",
        )
        caster_mat.func("/World/Materials/caster", caster_mat)
        for prim in self.scene.stage.Traverse():
            path = str(prim.GetPath())
            if not path.startswith("/World/envs/env_0/Robot") or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if "wheel" in path:
                sim_utils.bind_physics_material(path, "/World/Materials/wheel")
            elif "caster" in path:
                sim_utils.bind_physics_material(path, "/World/Materials/caster")
