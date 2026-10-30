"""S1·S2 공용 — 방 + 우리 로봇 + 카메라.

s01(보기), s02(키보드 주행)이 같은 장면을 쓰도록 한 곳에 둔다.
"""
from __future__ import annotations

import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[2]          # ros2/
ISAAC = WS / "isaac"
if str(ISAAC) not in sys.path:
    sys.path.insert(0, str(ISAAC))

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from tasks.common.robot_spec import robot_usd  # noqa: E402

WALL_H = 1.1
WALL_T = 0.05
ROBOT_Z = 0.03          # 스폰 높이 (정착 후 base ≈ 0.01)
CAM_EYE = (1.8, -1.8, 1.55)
CAM_TARGET = (0.0, 0.0, 0.15)


def _box(path: str, size, pos, color=None, static=True):
    cfg = sim_utils.CuboidCfg(
        size=size,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color) if color else None,
    )
    cfg.func(path, cfg, translation=pos)


def build_room(size: float):
    """바닥·조명·벽·기준 물체. size=0이면 빈 바닥만."""
    spawn_ground_plane("/World/ground", GroundPlaneCfg())
    light = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.95, 0.95, 0.95))
    light.func("/World/light", light, translation=(2.0, -2.0, 4.0))
    if size <= 0:
        return
    half = size / 2.0
    wall_color = (0.72, 0.72, 0.70)
    _box("/World/room/wall_px", (WALL_T, size, WALL_H), (half, 0.0, WALL_H / 2), wall_color)
    _box("/World/room/wall_nx", (WALL_T, size, WALL_H), (-half, 0.0, WALL_H / 2), wall_color)
    _box("/World/room/wall_py", (size, WALL_T, WALL_H), (0.0, half, WALL_H / 2), wall_color)
    _box("/World/room/wall_ny", (size, WALL_T, WALL_H), (0.0, -half, WALL_H / 2), wall_color)
    _box("/World/room/box_red", (0.3, 0.3, 0.3), (0.9, 0.7, 0.15), (0.75, 0.15, 0.12))
    _box("/World/room/box_green", (0.5, 0.25, 0.8), (-1.1, 0.9, 0.4), (0.12, 0.55, 0.20))
    _box("/World/room/box_blue", (0.2, 0.6, 0.2), (1.2, -1.0, 0.1), (0.15, 0.30, 0.75))


def set_camera(sim):
    sim.set_camera_view(eye=CAM_EYE, target=CAM_TARGET)


def spawn_robot():
    """우리 URDF → USD → /World/Robot. InteractiveScene 1개(복제 없음)."""
    usd_path = robot_usd()
    print(f"[scene] robot USD -> {usd_path}")

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        num_envs = 1
        env_spacing = 2.0

    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, ROBOT_Z)),
        actuators={"wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*wheel.*"], effort_limit=2.0,
            velocity_limit=8.0, stiffness=0.0, damping=40.0)},
    )
    scene = InteractiveScene(SceneCfg())
    robot = Articulation(robot_cfg)
    scene.articulations["robot"] = robot

    # 볼 캐스터 마찰 0 (RL 환경 _setup_scene과 같은 처리 — 안 하면 제자리 회전이 뻑뻑함)
    from pxr import UsdPhysics
    caster_mat = sim_utils.RigidBodyMaterialCfg(static_friction=0.0, dynamic_friction=0.0, friction_combine_mode="min")
    caster_mat.func("/World/Materials/caster", caster_mat)
    for prim in scene.stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith("/World/Robot") and "caster" in path and prim.HasAPI(UsdPhysics.CollisionAPI):
            sim_utils.bind_physics_material(path, "/World/Materials/caster")
    return scene, robot, usd_path


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
