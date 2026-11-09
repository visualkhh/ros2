"""rl_maze_beebot 학습자 — rl_maze와 같은 PPO, 실험 이름만 beebot."""
from __future__ import annotations

from isaaclab.utils import configclass

from agents.base_agent import PPO
from training.rl_maze.agent import RlMazeAgentCfg


@configclass
class RlMazeBeebotAgentCfg(RlMazeAgentCfg):
    experiment_name = "rl_maze_beebot"
    # 정책 출력이 [-1, 1] 밖으로 나가도 env가 clamp해 버려서, 그대로 두면 mean이 무한히 커진다.
    # 표준 clip_actions + entropy 0 + reward의 action 초과 패널티로 묶는다.
    clip_actions = 1.0
    algorithm = PPO.replace(learning_rate=3e-4, entropy_coef=0.0)
