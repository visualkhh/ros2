"""rl_maze — 미로 안에서 웨이포인트(경로)를 가장 빨리 따라가는 주행 정책 (nav2 controller 자리). Isaac Lab task 등록.

  RlMaze   env = RlMazeEnv · env_cfg = RlMazeEnvCfg · agent = RlMazeAgentCfg
  로봇: 이 폴더의 robot_cfg.py + usda/robot.usda (rl_correction_waypoint에서 의도적으로 복사) · 미로: usda/maze/
  학습: ./run-training.sh rl_maze  ·  재생: ./run-training.sh rl_maze --play --checkpoint <model_N.pt>
등록은 문자열 위치만 적는다 — 이 파일은 Isaac 앱 전에 import된다 (공식 스크립트 --external_callback).
"""
import gymnasium as gym

gym.register(
    id="RlMaze",
    entry_point=f"{__name__}.env:RlMazeEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:RlMazeEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agent:RlMazeAgentCfg",
    },
)
