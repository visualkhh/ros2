import sys

sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)

from isaaclab.sim import SimulationContext, SimulationCfg  # noqa: E402
from isaaclab.sim.spawners.from_files import UsdFileCfg  # noqa: E402
from isaaclab.sim.spawners.from_files.from_files import spawn_from_usd  # noqa: E402

USD = "/home/pank/workspaces/visualkhh/source/ros2/isaac/tasks/common/tb3_simple.usda"


def main():
    sim = SimulationContext(SimulationCfg(dt=1 / 120))
    spawn_from_usd("/World/Robot", UsdFileCfg(usd_path=USD))
    stage = sim.stage
    prim = stage.GetPrimAtPath("/World/Robot")
    print("valid:", prim.IsValid())
    print("children:", [str(c.GetPath()) for c in prim.GetChildren()])
    print("refs:", prim.GetMetadata("references"))
    print("hasPayload:", prim.HasPayload())
    sim.stop()


if __name__ == "__main__":
    main()
    app_launcher.app.close()
