"""rl_maze 학습자 설정 — 관측 51(라이다 포함) → 행동 2라 신경망을 조금 크게 (공통 PPO 표준값은 agents/base_agent.py)."""
from isaaclab.utils import configclass

from agents.base_agent import POLICY, PPO, BaseAgentCfg


@configclass
class RlMazeAgentCfg(BaseAgentCfg):
    experiment_name = "rl_maze"
    num_steps_per_env = 64       # 1회 학습에 env마다 모으는 스텝 (20 Hz → 3.2 s)
    max_iterations = 1000
    policy = POLICY.replace(
        init_noise_std=0.6,
        actor_hidden_dims=[128, 128],
        critic_hidden_dims=[128, 128],
    )
    algorithm = PPO.replace(
        learning_rate=3e-4,
        entropy_coef=0.005,
    )
