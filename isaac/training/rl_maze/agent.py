"""rl_maze 학습자 설정 — 관측 52(라이다 + inflation) → 행동 2라 신경망을 조금 크게 (공통 PPO 표준값은 agents/base_agent.py).

행동 분포는 Beta(-1,1) — Gaussian은 평균이 [-1,1] 밖으로 드리프트해도 env가 action을 clamp해서
PPO가 실제로 적용된 행동(±1)과 다른 raw 행동(수십~수천)으로 학습하는 불일치가 생긴다.
Beta는 표본·평균이 항상 [-1,1] 안이라 학습과 실기(ONNX mean)가 같은 행동을 쓴다.
"""
from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlMLPModelCfg

from agents.base_agent import POLICY, PPO, BaseAgentCfg


@configclass
class RlMazeAgentCfg(BaseAgentCfg):
    experiment_name = "rl_maze"
    num_steps_per_env = 64       # 1회 학습에 env마다 모으는 스텝 (20 Hz → 3.2 s)
    max_iterations = 1000
    # policy는 check()용으로만 남긴다 (rsl_rl 4.x는 아래 actor/critic을 쓴다).
    policy = POLICY.replace(
        init_noise_std=0.6,
        actor_hidden_dims=[128, 128],
        critic_hidden_dims=[128, 128],
    )
    actor = RslRlMLPModelCfg(
        hidden_dims=[128, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=RslRlMLPModelCfg.DistributionCfg(class_name="BetaDistribution"),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[128, 128],
        activation="elu",
        obs_normalization=False,
    )
    algorithm = PPO.replace(
        learning_rate=3e-4,
        entropy_coef=0.001,
    )
