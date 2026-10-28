import sys
sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, UsdFileCfg, spawn_ground_plane
from isaaclab.utils import configclass
USD = "/home/pank/workspaces/visualkhh/source/ros2/isaac/usd/beebot/beebot.usda"
sim = SimulationContext(SimulationCfg(dt=1 / 120))
spawn_ground_plane("/World/ground", GroundPlaneCfg())
@configclass
class SceneCfg(InteractiveSceneCfg):
    num_envs = 1
    env_spacing = 2.0
scene = InteractiveScene(SceneCfg())
print("A:", [str(p.GetPath()) for p in sim.stage.Traverse() if 'Robot' in str(p.GetPath())][:5])
cfg = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=UsdFileCfg(usd_path=USD),
    actuators={"wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_left_joint", "wheel_right_joint"],
              effort_limit=2.0, velocity_limit=8.0, stiffness=0.0, damping=40.0)},
)
robot = Articulation(cfg)
print("B:", [str(p.GetPath()) for p in sim.stage.Traverse() if 'Robot' in str(p.GetPath())][:5])
scene.articulations["robot"] = robot
print("C:", [str(p.GetPath()) for p in sim.stage.Traverse() if 'Robot' in str(p.GetPath())][:5])
try:
    sim.reset()
    print("RESET-OK")
except RuntimeError as e:
    print("RESET-FAIL:", str(e)[:100])
print("D:", [str(p.GetPath()) for p in sim.stage.Traverse() if 'Robot' in str(p.GetPath())][:5])
app_launcher.app.close()
