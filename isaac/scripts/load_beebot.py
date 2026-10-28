"""beebot 학습용 URDF headless load check (Isaac Sim)."""
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)

from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, UsdFileCfg, spawn_ground_plane  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

URDF = "/home/pank/workspaces/visualkhh/source/ros2/isaac/urdf/beebot/beebot.urdf"


def main():
    cfg = UrdfConverterCfg(
        asset_path=URDF,
        usd_dir="/tmp/beebot_usd",
        usd_file_name="beebot.usda",
        force_usd_conversion=True,
        fix_base=False,
        merge_fixed_joints=False,
    )
    usd_path = UrdfConverter(cfg).usd_path
    print(f"[beebot] usd -> {usd_path}")

    sim = SimulationContext(SimulationCfg(dt=1 / 120))
    spawn_ground_plane("/World/ground", GroundPlaneCfg())

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        num_envs = 1
        env_spacing = 2.0

    from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
    from isaaclab.assets import ArticulationCfg  # noqa: E402

    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=UsdFileCfg(usd_path=usd_path),
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
    print("[beebot] joints:", robot.joint_names)
    z = robot.data.root_pos_w.torch[0, 2].item()
    for _ in range(120):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
    z2 = robot.data.root_pos_w.torch[0, 2].item()
    print(f"[beebot] base height: {z:.3f} -> {z2:.3f} (fall = spawn/convert problem)")
    sim.stop()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
