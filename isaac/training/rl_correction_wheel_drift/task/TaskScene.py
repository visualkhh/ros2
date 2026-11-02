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
        slab_world = (getattr(c, "randomize_friction", False) or getattr(c, "randomize_uneven", False)
                      or getattr(c, "slope_deg", 0.0) > 0.0)
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
                q, on_slab = self._tilt(x, y, getattr(c, "slope_deg", 0.0))
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
                cfg.func(f"/World/grounds/ground_{i}", cfg, translation=(x, y, -0.05), orientation=q)

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
                    patch.func(path, patch, orientation=q,
                               translation=on_slab(radius * math.cos(angle), radius * math.sin(angle), -0.004))
                    sim_utils.bind_physics_material(f"{path}/geometry/mesh", mat_path)

                # 요철: 바퀴가 올라타는 낮은 턱. 기하학적 외란 (마찰과 독립).
                if getattr(c, "randomize_uneven", False):
                    half = size / 2.0
                    for j in range(int(c.uneven_patch_count)):
                        bw = random.uniform(*c.uneven_patch_size)
                        bd = random.uniform(*c.uneven_patch_size)
                        bh = random.uniform(*c.uneven_patch_height)
                        bx = random.uniform(-half + 0.35, half - 0.35)
                        by = random.uniform(-half + 0.35, half - 0.35)
                        bump = sim_utils.CuboidCfg(
                            size=(bw, bd, bh),
                            collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                            visual_material=sim_utils.PreviewSurfaceCfg(
                                diffuse_color=(0.55, 0.55, 0.58), roughness=0.9),
                        )
                        bump_path = f"/World/uneven/bump_{i}_{j}"
                        bump.func(bump_path, bump, translation=on_slab(bx, by, bh / 2.0), orientation=q)
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

    @staticmethod
    def _tilt(x: float, y: float, max_deg: float):
        """env 바닥 기울기 — 방향 랜덤 · 각도 0~max_deg. (quat xyzw, 판 위 점 → 월드 좌표 함수).

        판 중심(x, y, -0.05) 기준으로 돌려서 env 원점(로봇 스폰)의 바닥 높이는 거의 0 그대로.
        """
        th = math.radians(random.uniform(0.0, max_deg)) if max_deg > 0 else 0.0
        phi = random.uniform(0.0, 2 * math.pi)
        ax, ay = math.cos(phi), math.sin(phi)   # 수평 회전축
        s, co = math.sin(th), math.cos(th)
        q = (ax * math.sin(th / 2), ay * math.sin(th / 2), 0.0, math.cos(th / 2))

        def on_slab(dx: float, dy: float, z: float):
            # 평평할 때 월드 높이 z인 점 (판 중심 기준 dz = z + 0.05) → 축 (ax, ay, 0)으로 th 회전 (로드리게스)
            v = (dx, dy, z + 0.05)
            k_dot = ax * v[0] + ay * v[1]
            cross = (ay * v[2], -ax * v[2], ax * v[1] - ay * v[0])
            r = [v[i] * co + cross[i] * s + (ax, ay, 0.0)[i] * k_dot * (1 - co) for i in range(3)]
            return (x + r[0], y + r[1], -0.05 + r[2])
        return q, on_slab

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
