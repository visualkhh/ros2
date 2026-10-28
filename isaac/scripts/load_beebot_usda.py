"""GitHub beebot USDA direct load check (no URDF conversion)."""
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)

from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, UsdFileCfg, spawn_ground_plane  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

USD = "/home/pank/workspaces/visualkhh/source/ros2/isaac/usd/beebot/beebot.usda"


def main():
    sim = SimulationContext(SimulationCfg(dt=1 / 120))
    spawn_ground_plane("/World/ground", GroundPlaneCfg())

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        num_envs = 1
        env_spacing = 2.0

    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=UsdFileCfg(usd_path=USD, variants={"Physics": "physx"}),
        actuators={
            "wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_left_joint", "wheel_right_joint"],
                                          effort_limit=2.0, velocity_limit=8.0, stiffness=0.0, damping=40.0),
            "arm": ImplicitActuatorCfg(joint_names_expr=["joint1", "joint2", "joint3", "joint4", "joint5"],
                                       effort_limit=5.0, velocity_limit=3.0, stiffness=100.0, damping=10.0),
            "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_left_joint", "gripper_right_joint"],
                                           effort_limit=5.0, velocity_limit=0.5, stiffness=100.0, damping=10.0),
        },
    )
    scene = InteractiveScene(SceneCfg())
    robot = Articulation(robot_cfg)
    scene.articulations["robot"] = robot
    sim.reset()
    print("[usda] joints:", robot.joint_names)
    z = robot.data.root_pos_w.torch[0, 2].item()
    for _ in range(120):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
    z2 = robot.data.root_pos_w.torch[0, 2].item()
    print(f"[usda] base height: {z:.3f} -> {z2:.3f}")
    sim.stop()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
