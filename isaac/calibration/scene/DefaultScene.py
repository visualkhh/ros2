"""기본 장면: 벽 4개 + 장애물 방 + 우리 로봇 + 카메라. 한 파일 한 클래스."""
from __future__ import annotations

import math
import random
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[3]          # ros2/
ISAAC = WS / "isaac"
if str(ISAAC) not in sys.path:
    sys.path.insert(0, str(ISAAC))

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_from_euler_xyz  # noqa: E402

from calibration.parameter.SimParam import SimParam  # noqa: E402
from calibration.scene.Room import Room  # noqa: E402
from calibration.scene.Scene import Scene  # noqa: E402
from urdf.robot_spec import robot_usd  # noqa: E402


class DefaultScene(Scene):
    """S1·S2 공용 기본값. 방 구조·마찰을 바꾸려면 메서드를 오버라이딩한다."""

    def build(self, count: int = 1, size: float = 4.0, seed: int = 0, size_range=None,
              condition: str | None = None):
        cond = condition or self.condition
        f = SimParam.load(cond).friction
        obstacle_mat = self._material("/World/Materials/obstacle", f.obstacle)
        spawn_ground_plane("/World/ground", GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=f.ground[0], dynamic_friction=f.ground[1],
                restitution=0.0, friction_combine_mode="average"),
        ))
        light = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.95, 0.95, 0.95))
        light.func("/World/light", light, translation=(2.0, -2.0, 4.0))

        self.rooms = []
        for i in range(count):
            room_size = size if size_range is None else random.Random(seed + i * 7919).uniform(*size_range)
            origin = (i * self.ROOM_SPACING, 0.0, 0.0)
            self.rooms.append(self._build_room(i, origin, room_size, seed + i * 1000, obstacle_mat))
        return self.rooms

    def set_camera(self, sim, origin=(0.0, 0.0, 0.0)):
        ox, oy, _ = origin
        sim.set_camera_view(eye=(ox + self.CAM_OFF[0], oy + self.CAM_OFF[1], self.CAM_OFF[2]),
                            target=(ox, oy, self.CAM_LOOK_Z))

    def articulation_cfg(self, prim_path: str = "/World/Robot", usd_path: str | None = None,
                         fix_root: bool = False, init_z: float | None = None,
                         condition: str | None = None):
        cond = condition or self.condition
        usd_path = usd_path or robot_usd()
        act = self.actuator or SimParam.load(cond).actuator
        cfg = ArticulationCfg(
            prim_path=prim_path,
            spawn=sim_utils.UsdFileCfg(usd_path=usd_path, fix_root_link=True if fix_root else None),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, self.ROBOT_Z if init_z is None else init_z)),
            actuators={"wheels": ImplicitActuatorCfg(
                joint_names_expr=[".*wheel.*"], effort_limit=act.effort,
                velocity_limit=act.velocity_limit, stiffness=0.0, damping=act.damping)},
        )
        return cfg, usd_path

    def bind_robot_materials(self, scene=None, prim_path: str = "/World/Robot",
                             condition: str | None = None):
        from pxr import PhysxSchema, UsdPhysics
        scene = scene if scene is not None else self.scene
        cond = condition or self.condition
        f = SimParam.load(cond).friction
        wheel_mat = self._material("/World/Materials/wheel", f.wheel)
        caster_mat = self._material("/World/Materials/caster", f.caster)
        for prim in scene.stage.Traverse():
            path = str(prim.GetPath())
            if not path.startswith(prim_path):
                continue
            # URDF importer는 generic PhysicsRigidBodyAPI만 넣는다. PhysX 센서(IMU/RayCaster)의
            # rigid body view가 PhysxRigidBodyAPI를 요구하므로 붙여준다.
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if "wheel" in path:
                sim_utils.bind_physics_material(path, wheel_mat)
            elif "caster" in path:
                sim_utils.bind_physics_material(path, caster_mat)

    def spawn(self, airborne: bool = False, init_z: float | None = None,
              condition: str | None = None):
        cond = condition or self.condition
        robot_cfg, usd_path = self.articulation_cfg(
            fix_root=airborne, condition=cond,
            init_z=init_z if init_z is not None else (0.5 if airborne else self.ROBOT_Z))
        print(f"[{self.TAG}] robot USD -> {usd_path}")

        @configclass
        class SceneCfg(InteractiveSceneCfg):
            num_envs = 1
            env_spacing = 2.0

        self.scene = InteractiveScene(SceneCfg())
        self.robot = Articulation(robot_cfg)
        self.scene.articulations["robot"] = self.robot
        self.bind_robot_materials(condition=cond)
        self.usd_path = usd_path
        return self.scene, self.robot, usd_path

    def place(self, origin, yaw: float = 0.0, wheel_ids=None, robot=None):
        """로봇을 방 origin으로 순간이동 (속도·바퀴 상태 0). robot을 주면 그 로봇을 옮긴다."""
        robot = robot if robot is not None else self.robot
        dev = robot.device
        root = robot.data.default_root_state.torch.clone()
        root[:, 0] = origin[0]
        root[:, 1] = origin[1]
        root[:, 2] = self.ROBOT_Z
        zero = torch.zeros(1, device=dev)
        root[:, 3:7] = quat_from_euler_xyz(zero, zero, torch.tensor([yaw], device=dev))
        robot.write_root_pose_to_sim(root[:, :7])
        robot.write_root_velocity_to_sim(torch.zeros((1, 6), device=dev))
        if wheel_ids is not None:
            z = torch.zeros((1, len(wheel_ids)), device=dev)
            robot.write_joint_state_to_sim(z, z, wheel_ids)
            robot.set_joint_velocity_target(z, joint_ids=wheel_ids)

    def settle(self, sim, steps: int = 120, scene=None, robot=None):
        """스폰 높이에서 떨어져 바퀴·캐스터로 서게. 1초. 커스텀 scene/robot도 받는다.

        렌더는 마지막 한 번만 — 매 스텝 그리면 시작이 몇 초씩 느려진다.
        """
        scene = scene if scene is not None else self.scene
        robot = robot if robot is not None else self.robot
        dt = sim.get_physics_dt()
        for i in range(steps):
            scene.write_data_to_sim()
            sim.step(render=(i == steps - 1))
            scene.update(dt)
        return robot.data.root_pos_w.torch[0, 2].item()

    @staticmethod
    def _material(path: str, friction, restitution: float = 0.0):
        cfg = sim_utils.RigidBodyMaterialCfg(
            static_friction=friction[0], dynamic_friction=friction[1],
            restitution=restitution, friction_combine_mode="average",
        )
        cfg.func(path, cfg)
        return path

    @staticmethod
    def _visual(color):
        return sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.6)

    @staticmethod
    def _collision():
        # PhysX 콜라이더 (deprecated 아닌 쪽). 이게 없으면 그냥 그림이라 통과한다.
        return sim_utils.PhysxCollisionPropertiesCfg()

    @classmethod
    def _box(cls, path: str, size, pos, color=None, yaw: float = 0.0, material: str | None = None):
        cfg = sim_utils.CuboidCfg(
            size=size,
            collision_props=cls._collision(),
            visual_material=cls._visual(color) if color else None,
        )
        quat = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) if yaw else None
        prim = cfg.func(path, cfg, translation=pos, orientation=quat)
        if material:
            sim_utils.bind_physics_material(f"{path}/geometry/mesh", material)
        return prim

    @classmethod
    def _cylinder(cls, path: str, radius, height, pos, color, material: str | None = None):
        cfg = sim_utils.CylinderCfg(
            radius=radius, height=height,
            collision_props=cls._collision(),
            visual_material=cls._visual(color),
        )
        cfg.func(path, cfg, translation=pos)
        if material:
            sim_utils.bind_physics_material(f"{path}/geometry/mesh", material)

    @classmethod
    def _sample_obstacles(cls, rng: random.Random, half: float, count: int):
        """방 안에 겹치지 않게 장애물 배치. 로봇 자리(원점 0.75 m)는 비운다."""
        placed = []   # (x, y, 반경) — 겹침 검사용
        specs = []    # (kind, dims, x, y)
        for _ in range(count):
            for _try in range(200):
                if rng.random() < 0.72:
                    w, d, h = rng.uniform(0.20, 0.50), rng.uniform(0.20, 0.50), rng.uniform(0.18, 0.80)
                    rad = 0.5 * math.hypot(w, d)
                    kind, dims = "box", (w, d, h)
                else:
                    rad, h = rng.uniform(0.10, 0.22), rng.uniform(0.30, 0.80)
                    kind, dims = "cyl", (rad, h)
                x = rng.uniform(-half + 0.35, half - 0.35)
                y = rng.uniform(-half + 0.35, half - 0.35)
                if math.hypot(x, y) < cls.FREE_RADIUS + rad:
                    continue
                if any(math.hypot(x - px, y - py) < rad + pr + 0.25 for px, py, pr in placed):
                    continue
                placed.append((x, y, rad))
                specs.append((kind, dims, x, y))
                break
        return specs

    @classmethod
    def _build_room(cls, index: int, origin, size: float, seed: int, obstacle_mat: str) -> Room:
        half = size / 2.0
        ox, oy, _ = origin
        tag = f"/World/rooms/room_{index}"
        wall_color = (0.72, 0.72, 0.70)
        cls._box(f"{tag}/wall_px", (cls.WALL_T, size, cls.WALL_H), (ox + half, oy, cls.WALL_H / 2), wall_color)
        cls._box(f"{tag}/wall_nx", (cls.WALL_T, size, cls.WALL_H), (ox - half, oy, cls.WALL_H / 2), wall_color)
        cls._box(f"{tag}/wall_py", (size, cls.WALL_T, cls.WALL_H), (ox, oy + half, cls.WALL_H / 2), wall_color)
        cls._box(f"{tag}/wall_ny", (size, cls.WALL_T, cls.WALL_H), (ox, oy - half, cls.WALL_H / 2), wall_color)

        rng = random.Random(seed)
        n = rng.randint(3, 6)
        obstacles = cls._sample_obstacles(rng, half, n)
        for j, (kind, dims, x, y) in enumerate(obstacles):
            color = cls.PALETTE[(index * 3 + j) % len(cls.PALETTE)]
            if kind == "box":
                w, d, h = dims
                cls._box(f"{tag}/obstacle_{j}", (w, d, h), (ox + x, oy + y, h / 2),
                         color, yaw=rng.uniform(-math.pi, math.pi), material=obstacle_mat)
            else:
                rad, h = dims
                cls._cylinder(f"{tag}/obstacle_{j}", rad, h, (ox + x, oy + y, h / 2), color, material=obstacle_mat)
        return Room(index=index, origin=origin, seed=seed, obstacles=len(obstacles), size=size)
