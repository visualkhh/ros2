"""S1·S2 공용 — 방(들) + 우리 로봇 + 카메라.

- 방은 벽 4개 + 장애물 3~6개. 장애물·벽에 **콜라이더**를 붙인다 (안 붙이면 통과함).
- 마찰: 바닥 0.8/0.7 · 장애물 0.7/0.5 · 바퀴 1.0/1.0 · 캐스터 0/0.
- 여러 방을 x축으로 8 m 간격으로 깔아 두고, 로봇만 순간이동시켜 "랜덤 환경 다시"를 만든다.
  (USD 프림을 지웠다 다시 만들지 않아 안전·즉시)
"""
from __future__ import annotations

import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

WS = Path(__file__).resolve().parents[2]          # ros2/
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

from tasks.common.robot_spec import robot_usd  # noqa: E402

WALL_H = 1.1
WALL_T = 0.05
ROBOT_Z = 0.03
ROOM_SPACING = 8.0      # 방 사이 [m] — 라이다(≈3.5 m)가 옆방을 못 보게
FREE_RADIUS = 0.75      # 로봇 스폰 자리 — 장애물 금지
CAM_OFF = (1.8, -1.8, 1.55)
CAM_LOOK_Z = 0.15

# 마찰 (static, dynamic). 실제 Waffle Pi 바퀴는 고무, 바닥은 연구실 장판 기준의 출발값.
FRICTION_GROUND = (0.8, 0.7)
FRICTION_OBSTACLE = (0.7, 0.5)
FRICTION_WHEEL = (1.0, 1.0)

PALETTE = (
    (0.75, 0.15, 0.12), (0.12, 0.55, 0.20), (0.15, 0.30, 0.75),
    (0.75, 0.55, 0.10), (0.45, 0.20, 0.60), (0.20, 0.60, 0.65),
)


@dataclass
class Room:
    index: int
    origin: tuple[float, float, float]
    seed: int
    obstacles: int


def _material(path: str, friction, restitution: float = 0.0):
    cfg = sim_utils.RigidBodyMaterialCfg(
        static_friction=friction[0], dynamic_friction=friction[1],
        restitution=restitution, friction_combine_mode="average",
    )
    cfg.func(path, cfg)
    return path


def _visual(color):
    return sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.6)


def _collision():
    # PhysX 콜라이더 (deprecated 아닌 쪽). 이게 없으면 그냥 그림이라 통과한다.
    return sim_utils.PhysxCollisionPropertiesCfg()


def _box(path: str, size, pos, color=None, yaw: float = 0.0, material: str | None = None):
    cfg = sim_utils.CuboidCfg(
        size=size,
        collision_props=_collision(),
        visual_material=_visual(color) if color else None,
    )
    quat = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) if yaw else None
    prim = cfg.func(path, cfg, translation=pos, orientation=quat)
    if material:
        sim_utils.bind_physics_material(f"{path}/geometry/mesh", material)
    return prim


def _cylinder(path: str, radius, height, pos, color, material: str | None = None):
    cfg = sim_utils.CylinderCfg(
        radius=radius, height=height,
        collision_props=_collision(),
        visual_material=_visual(color),
    )
    cfg.func(path, cfg, translation=pos)
    if material:
        sim_utils.bind_physics_material(f"{path}/geometry/mesh", material)


def _sample_obstacles(rng: random.Random, half: float, count: int):
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
            if math.hypot(x, y) < FREE_RADIUS + rad:
                continue
            if any(math.hypot(x - px, y - py) < rad + pr + 0.25 for px, py, pr in placed):
                continue
            placed.append((x, y, rad))
            specs.append((kind, dims, x, y))
            break
    return specs


def _build_room(index: int, origin, size: float, seed: int, obstacle_mat: str) -> Room:
    half = size / 2.0
    ox, oy, _ = origin
    tag = f"/World/rooms/room_{index}"
    wall_color = (0.72, 0.72, 0.70)
    _box(f"{tag}/wall_px", (WALL_T, size, WALL_H), (ox + half, oy, WALL_H / 2), wall_color)
    _box(f"{tag}/wall_nx", (WALL_T, size, WALL_H), (ox - half, oy, WALL_H / 2), wall_color)
    _box(f"{tag}/wall_py", (size, WALL_T, WALL_H), (ox, oy + half, WALL_H / 2), wall_color)
    _box(f"{tag}/wall_ny", (size, WALL_T, WALL_H), (ox, oy - half, WALL_H / 2), wall_color)

    rng = random.Random(seed)
    n = rng.randint(3, 6)
    obstacles = _sample_obstacles(rng, half, n)
    for j, (kind, dims, x, y) in enumerate(obstacles):
        color = PALETTE[(index * 3 + j) % len(PALETTE)]
        if kind == "box":
            w, d, h = dims
            _box(f"{tag}/obstacle_{j}", (w, d, h), (ox + x, oy + y, h / 2),
                 color, yaw=rng.uniform(-math.pi, math.pi), material=obstacle_mat)
        else:
            rad, h = dims
            _cylinder(f"{tag}/obstacle_{j}", rad, h, (ox + x, oy + y, h / 2), color, material=obstacle_mat)
    return Room(index=index, origin=origin, seed=seed, obstacles=len(obstacles))


def build_world(count: int = 1, size: float = 4.0, seed: int = 0):
    """바닥·조명·방 count개. 반환: [Room]. count=0이면 빈 바닥만."""
    obstacle_mat = _material("/World/Materials/obstacle", FRICTION_OBSTACLE)
    spawn_ground_plane("/World/ground", GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=FRICTION_GROUND[0], dynamic_friction=FRICTION_GROUND[1],
            restitution=0.0, friction_combine_mode="average"),
    ))
    light = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.95, 0.95, 0.95))
    light.func("/World/light", light, translation=(2.0, -2.0, 4.0))

    rooms = []
    for i in range(count):
        origin = (i * ROOM_SPACING, 0.0, 0.0)
        rooms.append(_build_room(i, origin, size, seed + i * 1000, obstacle_mat))
    return rooms


def set_camera(sim, origin=(0.0, 0.0, 0.0)):
    ox, oy, _ = origin
    sim.set_camera_view(eye=(ox + CAM_OFF[0], oy + CAM_OFF[1], CAM_OFF[2]),
                        target=(ox, oy, CAM_LOOK_Z))


def robot_articulation_cfg(prim_path: str = "/World/Robot", usd_path: str | None = None):
    """로봇 ArticulationCfg + USD 경로. L3처럼 센서를 같이 넣는 scene에서 재사용."""
    usd_path = usd_path or robot_usd()
    cfg = ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, ROBOT_Z)),
        actuators={"wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*wheel.*"], effort_limit=3.0,   # XM430-W210 stall ≈ 3 N·m (RL 환경과 같게)
            velocity_limit=8.0, stiffness=0.0, damping=20.0)},
    )
    return cfg, usd_path


def bind_robot_materials(scene, prim_path: str = "/World/Robot"):
    """바퀴 마찰 1.0 · 캐스터 0 (RL 환경 _setup_scene과 같은 처리)."""
    from pxr import PhysxSchema, UsdPhysics
    wheel_mat = _material("/World/Materials/wheel", FRICTION_WHEEL)
    caster_mat = _material("/World/Materials/caster", (0.0, 0.0))
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


def spawn_robot():
    """우리 URDF → USD → /World/Robot. InteractiveScene 1개(복제 없음)."""
    robot_cfg, usd_path = robot_articulation_cfg()
    print(f"[scene] robot USD -> {usd_path}")

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        num_envs = 1
        env_spacing = 2.0

    scene = InteractiveScene(SceneCfg())
    robot = Articulation(robot_cfg)
    scene.articulations["robot"] = robot
    bind_robot_materials(scene)
    return scene, robot, usd_path


def place_robot(robot, origin, yaw: float = 0.0, wheel_ids=None):
    """로봇을 방 origin으로 순간이동 (속도·바퀴 상태 0)."""
    dev = robot.device
    root = robot.data.default_root_state.torch.clone()
    root[:, 0] = origin[0]
    root[:, 1] = origin[1]
    root[:, 2] = ROBOT_Z
    zero = torch.zeros(1, device=dev)
    root[:, 3:7] = quat_from_euler_xyz(zero, zero, torch.tensor([yaw], device=dev))
    robot.write_root_pose_to_sim(root[:, :7])
    robot.write_root_velocity_to_sim(torch.zeros((1, 6), device=dev))
    if wheel_ids is not None:
        z = torch.zeros((1, len(wheel_ids)), device=dev)
        robot.write_joint_state_to_sim(z, z, wheel_ids)
        robot.set_joint_velocity_target(z, joint_ids=wheel_ids)


def settle(scene, robot, sim, steps: int = 120):
    """스폰 높이에서 떨어져 바퀴·캐스터로 서게. 1초.

    렌더는 마지막 한 번만 — 매 스텝 그리면 시작이 몇 초씩 느려진다.
    """
    dt = sim.get_physics_dt()
    for i in range(steps):
        scene.write_data_to_sim()
        sim.step(render=(i == steps - 1))
        scene.update(dt)
    return robot.data.root_pos_w.torch[0, 2].item()
