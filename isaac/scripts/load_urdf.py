"""URDF headless load check: xacro -> URDF -> USD -> spawn -> step 200 frames.

Usage (Lab venv):
  ./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/load_urdf.py
Prints joint/link list + base height (should stay ~0.01, not fall).
"""
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
from isaaclab.sim import SimulationContext  # noqa: E402

import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg  # noqa: E402
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, UsdFileCfg, spawn_ground_plane  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from tasks.common.robot_spec import robot_usd  # noqa: E402


def main():
    usd_path = robot_usd()
    print(f"[urdf] usd -> {usd_path}")

    sim_cfg = SimulationCfg(dt=1 / 120)
    sim = SimulationContext(sim_cfg)
    spawn_ground_plane("/World/ground", GroundPlaneCfg())

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        num_envs = 1
        env_spacing = 2.0

    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=UsdFileCfg(usd_path=usd_path),
        actuators={"wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*wheel.*"], effort_limit=2.0,
            velocity_limit=8.0, stiffness=0.0, damping=40.0)},
    )
    scene = InteractiveScene(SceneCfg())
    robot = Articulation(robot_cfg)
    scene.articulations["robot"] = robot
    sim.reset()
    print("[urdf] joints:", robot.joint_names)
    print("[urdf] links:", robot.body_names)

    sim_dt = sim.get_physics_dt()
    for _ in range(200):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
    z = robot.data.root_pos_w[0, 2].item()
    print(f"[urdf] base height after 200 steps: {z:.4f} m (expect ~0.01, fall = spawn/convert problem)")
    sim.stop()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
