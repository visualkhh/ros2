import sys
sys.path.insert(0, "/home/pank/workspaces/visualkhh/source/ros2/isaac")
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
import torch
from tasks.rl_correct.rl_correct_env_cfg import RlCorrectEnvCfg
from tasks.rl_correct.rl_correct_env import RlCorrectEnv

def main():
    cfg = RlCorrectEnvCfg()
    cfg.scene.num_envs = 8
    env = RlCorrectEnv(cfg)
    obs, _ = env.reset()
    for i in range(500):
        act = torch.randn((8, 2), device=env.device)
        obs, rew, term, tout, _ = env.step(act)
        o = obs["policy"]
        if bool(torch.isnan(o).any()):
            bad = torch.isnan(o).any(dim=0)
            print(f"[nan] step {i} cols:", bad.tolist())
            print("[nan] sample:", o[0].tolist())
            print("[nan] pos z:", env.robot.data.root_pos_w.torch[:, 2].tolist())
            break
        if (i + 1) % 100 == 0:
            print(f"[ok] step {i+1} rew={float(rew.mean()):.2f} zmin={float(env.robot.data.root_pos_w.torch[:,2].min()):.3f}")
    else:
        print("[ok] 500 steps clean")
    env.close()

if __name__ == "__main__":
    main()
    app_launcher.app.close()
