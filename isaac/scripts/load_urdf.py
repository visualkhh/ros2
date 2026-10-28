"""URDF headless load check: xacro -> URDF -> USD -> spawn -> step 200 frames.

Usage (Lab venv):
  ./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/load_urdf.py
Prints joint/link list + base height (should stay ~0.01, not fall).
"""
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
from isaaclab.sim import SimulationContext  # noqa: E402

import os

import xacro
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, UsdFileCfg, spawn_ground_plane
from isaaclab.utils import configclass

WS = "/home/pank/workspaces/visualkhh/source/ros2"
XACRO = WS + "/ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro"
TMP_URDF = "/tmp/tb3_waffle_pi.urdf"
USD_DIR = "/tmp/tb3_usd"


def main():
    doc = xacro.process_file(XACRO)
    with open(TMP_URDF, "w") as f:
        f.write(doc.toprettyxml(indent="  "))
    print(f"[urdf] xacro -> {TMP_URDF} ({os.path.getsize(TMP_URDF)} bytes)")

    cfg = UrdfConverterCfg(
        asset_path=TMP_URDF,
        usd_dir=USD_DIR,
        usd_file_name="tb3_waffle_pi.usd",
        force_usd_conversion=True,
        fix_base=False,
        merge_fixed_joints=False,
        joint_drive=None,
        ros_package_paths=[{"name": "tb3_bringup", "path": WS + "/ros2_ws/src/tb3_bringup"}],
    )
    converter = UrdfConverter(cfg)
    usd_path = converter.usd_path
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
