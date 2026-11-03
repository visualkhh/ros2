"""BaseAgentCfg — 우리 공통 PPO 기본값 (RSL-RL, Isaac Lab 6.1 RslRlOnPolicyRunnerCfg).

  policy     정책(actor) · 가치(critic) 신경망 — actor가 policy.onnx로 나간다
  algorithm  PPO 학습 방법 (학습률 · 탐색 보너스 · 한 번에 얼마나 바꿀지)
  그 밖      env마다 모으는 스텝 · 학습 횟수 · 저장 간격 · 로그
"""
from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class BaseAgentCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 64      # 1회 학습에 env마다 모으는 스텝
    max_iterations = 300        # 기본 학습 횟수 (run-training.sh --iterations)
    save_interval = 50          # model_<N>.pt 저장 간격
    experiment_name = "agent"
    logger = "tensorboard"
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.5,          # 처음 탐색 노이즈
        actor_hidden_dims=[64, 64],  # 정책 신경망 (→ policy.onnx, 약 20 KB)
        critic_hidden_dims=[64, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,          # 탐색 보너스 — 신호가 약하면 노이즈가 커지는 쪽으로 밀 수 있다 (2026-11-02 8000회 폭주)
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
