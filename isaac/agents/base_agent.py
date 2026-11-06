"""BaseAgentCfg — 학습자(agent) 설정의 바탕. **task와 상관없는 PPO 표준값만** (Isaac Lab rsl_rl 예제들의 공통값).

task마다 맞춰야 하는 값은 비워 둔다 (MISSING — task agent.py가 안 정하면 학습 시작 때 에러):
  num_steps_per_env · max_iterations · experiment_name
  policy: init_noise_std · actor_hidden_dims · critic_hidden_dims
  algorithm: learning_rate · entropy_coef
task는 `policy = POLICY.replace(...)` · `algorithm = PPO.replace(...)`로 채운다. 빠지면 task cfg가 만들어질 때 check()가 멈춘다.
"""
from dataclasses import MISSING

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

# 정책(actor) · 가치(critic) 신경망 — 크기 · 초기 노이즈는 task가
POLICY = RslRlPpoActorCriticCfg(activation="elu")

# PPO 학습 방법 — 학습률 · 탐색 보너스는 task가
PPO = RslRlPpoAlgorithmCfg(
    value_loss_coef=1.0,
    use_clipped_value_loss=True,
    clip_param=0.2,              # 한 번에 정책을 너무 크게 안 바꾸게
    num_learning_epochs=5,
    num_mini_batches=4,
    schedule="adaptive",         # desired_kl 보고 학습률 자동 조절
    gamma=0.99,                  # 미래 보상 할인
    lam=0.95,                    # GAE
    desired_kl=0.01,
    max_grad_norm=1.0,
)


# task agent.py가 꼭 채워야 하는 값 (task cfg가 만들어질 때 check()로 확인)
REQUIRED = ("experiment_name", "num_steps_per_env", "max_iterations",
            "policy.init_noise_std", "policy.actor_hidden_dims", "policy.critic_hidden_dims",
            "algorithm.learning_rate", "algorithm.entropy_coef")


def check(cfg) -> None:
    """REQUIRED 중 안 채운 값이 있으면 이름을 모아 에러 (Isaac Lab validate()는 호환 변환이 나중에 채우는 칸까지 잡아서 못 씀)."""
    def get(path):
        v = cfg
        for k in path.split("."):
            v = getattr(v, k)
        return v
    missing = [p for p in REQUIRED if isinstance(get(p), type(MISSING))]   # configclass가 MISSING을 복사해서 is로는 안 잡힘
    if missing:
        raise TypeError(f"{type(cfg).__name__}: task agent.py에서 정해야 하는 값 — {', '.join(missing)}")


@configclass
class BaseAgentCfg(RslRlOnPolicyRunnerCfg):
    save_interval = 50           # model_<N>.pt 저장 간격
    logger = "tensorboard"
    # rsl_rl 4.x: env observation group 이름("policy")을 actor/critic에 명시한다.
    #   비워두면 rsl_rl이 "policy를 actor/critic으로 가정"한다는 경고를 낸다 (동작은 함).
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}
    policy = POLICY
    algorithm = PPO

    def __post_init__(self):
        if type(self) is not BaseAgentCfg:   # task cfg가 만들어질 때 (공식 학습 스크립트가 만든다)
            check(self)
