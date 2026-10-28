"""GUI window test: opens Isaac Sim with one cube for 15 seconds."""
import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher  # noqa: E402

import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
print("[win] headless =", args.headless if hasattr(args, "headless") else "(no flag)")
app_launcher = AppLauncher(args)

import time  # noqa: E402

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane  # noqa: E402
from isaaclab.sim.spawners.shapes import CuboidCfg, spawn_cuboid  # noqa: E402

sim = SimulationContext(SimulationCfg(dt=1 / 60))
spawn_ground_plane("/World/ground", GroundPlaneCfg())
spawn_cuboid("/World/box", CuboidCfg(size=(0.5, 0.5, 0.5)), translation=(0, 0, 1.0))
sim.reset()
print("[win] window should be visible now (cube falling). waiting 15s...")
for _ in range(900):
    sim.step()
    time.sleep(1 / 60)
    if app_launcher.app.is_exiting():
        break
print("[win] done")
app_launcher.app.close()
