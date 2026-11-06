"""rl_maze_beebot — beebot 미로 주행 task 등록. (미로·보상·동적 장애물은 rl_maze 그대로)

  RlMazeBeebot  env = RlMazeBeebotEnv · env_cfg = RlMazeBeebotEnvCfg · agent = RlMazeBeebotAgentCfg
  로봇: 이 폴더의 robot_cfg.py + usda/robot.usda (beebot, Lv2_bee_Assignment에서 복사)
  센서: beebot pole_front_cam point cloud와 같은 36방향 가상 스캔 (55° FOV 밖은 lidar_max)
  학습: ./run-training.sh rl_maze_beebot
"""
import gymnasium as gym

gym.register(
    id="RlMazeBeebot",
    entry_point=f"{__name__}.env:RlMazeBeebotEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:RlMazeBeebotEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agent:RlMazeBeebotAgentCfg",
    },
)
