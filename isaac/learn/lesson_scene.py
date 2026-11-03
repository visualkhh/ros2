"""강의 공용 장면 — Isaac Lab 튜토리얼(create_scene.py) 관례: InteractiveSceneCfg에 바닥 · 조명 · 로봇을 선언.

  scene = InteractiveScene(TB3SceneCfg(num_envs=1, env_spacing=2.0))   # 센서를 더하려면 상속 (l03)
  rooms(1, 4.0)           # 방 + 장애물 (calibration env와 같은 생성기) — InteractiveScene 전에
AppLauncher 뒤에 import한다 (로봇 USD 변환에 Isaac 앱이 필요).
"""
from __future__ import annotations

from types import SimpleNamespace

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_from_euler_xyz

from assets.tb3_waffle_pi import GROUND_FRICTION, OBSTACLE_FRICTION, TB3_CFG, bind_wheel_materials
from calibration.calib_env import ROBOT_Z, spawn_rooms


@configclass
class TB3SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=GroundPlaneCfg(physics_material=sim_utils.RigidBodyMaterialCfg(
        static_friction=GROUND_FRICTION[0], dynamic_friction=GROUND_FRICTION[1], restitution=0.0,
        friction_combine_mode="average")))
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(0.95, 0.95, 0.95)))
    robot = TB3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def rooms(count: int, size: float, seed: int = 0) -> list[dict]:
    return spawn_rooms(SimpleNamespace(rooms=count, room_size=(size, size), room_seed=seed, room_spacing=8.0,
                                       obstacle_friction=OBSTACLE_FRICTION))


def ready(sim, scene, steps: int = 120) -> float:
    """바퀴 재질 → sim.reset → 스폰 높이에서 떨어져 서게 (렌더는 마지막만). 반환: 몸체 높이 [m]."""
    bind_wheel_materials(scene.stage, "/World/envs/env_0/Robot")
    sim.reset()
    scene.reset()
    dt = sim.get_physics_dt()
    for i in range(steps):
        scene.write_data_to_sim()
        sim.step(render=(i == steps - 1))
        scene.update(dt)
    return scene["robot"].data.root_pos_w.torch[0, 2].item()


def place(robot, origin=(0.0, 0.0, 0.0), yaw: float = 0.0, wheel_ids=None) -> None:
    """env 0 로봇을 origin으로 순간이동 (속도 · 바퀴 0)."""
    dev = robot.device
    root = robot.data.default_root_state.torch[:1].clone()
    root[:, 0], root[:, 1], root[:, 2] = origin[0], origin[1], ROBOT_Z
    zero = torch.zeros(1, device=dev)
    root[:, 3:7] = quat_from_euler_xyz(zero, zero, torch.tensor([yaw], device=dev))
    robot.write_root_pose_to_sim(root[:, :7])
    robot.write_root_velocity_to_sim(torch.zeros((1, 6), device=dev))
    if wheel_ids is not None:
        z = torch.zeros((1, len(wheel_ids)), device=dev)
        robot.write_joint_state_to_sim(z, z, wheel_ids)
        robot.set_joint_velocity_target(z, joint_ids=wheel_ids)
