"""rl_correction_wheel_drift 학습자 설정 — 이 task에 맞춘 값 (공통 PPO 표준값은 agents/base_agent.py)."""
from isaaclab.utils import configclass

from agents.base_agent import POLICY, PPO, BaseAgentCfg


@configclass
class RlCorrectAgentCfg(BaseAgentCfg):
    experiment_name = "rl_correct"
    num_steps_per_env = 64       # 1회 학습에 env마다 모으는 스텝 (20 Hz → 3.2 s)
    max_iterations = 300         # 기본 학습 횟수 (run-training.sh --iterations로 바꿈)
    policy = POLICY.replace(
        init_noise_std=0.5,          # 처음 탐색 노이즈 (행동 [-1, 1] 기준)
        actor_hidden_dims=[64, 64],  # 정책 신경망 — 관측 6 → 행동 2라 작게 (→ policy.onnx 약 20 KB)
        critic_hidden_dims=[64, 64],
    )
    algorithm = PPO.replace(
        learning_rate=3e-4,
        entropy_coef=0.005,          # 탐색 보너스 — 신호가 약하면 노이즈가 커지는 쪽으로 밀 수 있다 (2026-11-02 8000회 폭주)
    )
