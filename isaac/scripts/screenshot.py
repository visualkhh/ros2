"""Offscreen proof: Lab Camera captures the TB3 robot, saves PNG."""
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sensors import CameraCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
import isaaclab.sim as _sim_utils  # noqa: E402
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, UsdFileCfg, spawn_ground_plane  # noqa: E402
from isaaclab.sim.spawners.from_files.from_files import spawn_from_usd  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

USD = "/home/pank/workspaces/visualkhh/source/ros2/isaac/tasks/common/tb3_simple.usda"


@configclass
class ShotSceneCfg(InteractiveSceneCfg):
    num_envs = 1
    env_spacing = 2.0
    camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera",
        update_period=0,
        height=480,
        width=640,
        data_types=["rgb"],
        spawn=_sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
        ),
        offset=CameraCfg.OffsetCfg(pos=(-1.5, 0.0, 1.0), rot=(0.9239, 0.0, 0.3827, 0.0), convention="world"),
    )


def main():
    sim = SimulationContext(SimulationCfg(dt=1 / 120))
    spawn_ground_plane("/World/ground", GroundPlaneCfg())
    spawn_from_usd("/World/envs/env_0/Robot", UsdFileCfg(usd_path=USD))
    scene = InteractiveScene(ShotSceneCfg())
    sim.reset()
    scene.reset()
    for _ in range(30):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
    rgb = scene["camera"].data.output["rgb"][0].cpu().numpy()
    out = "/tmp/isaac_shot.png"
    Image.fromarray(rgb.astype(np.uint8)).save(out)
    print(f"[shot] saved {out} shape={rgb.shape}")
    sim.stop()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
