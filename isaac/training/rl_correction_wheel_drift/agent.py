"""이 task의 학습자 설정 — 공통 바탕(agents/base_agent.py)에서 다른 값만."""
from isaaclab.utils import configclass

from agents.base_agent import BaseAgentCfg


@configclass
class RlCorrectAgentCfg(BaseAgentCfg):
    experiment_name = "rl_correct"
