"""rl_maze_beebot 학습자 — rl_maze와 같은 PPO, 실험 이름만 beebot."""
from __future__ import annotations

from isaaclab.utils import configclass

from training.rl_maze.agent import RlMazeAgentCfg


@configclass
class RlMazeBeebotAgentCfg(RlMazeAgentCfg):
    experiment_name = "rl_maze_beebot"
