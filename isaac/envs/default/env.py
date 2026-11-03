"""DefaultEnv — 아무 규칙 없는 기본 세상. BaseEnv가 다 해 준다 ([v, w] → 드라이버 식 → 바퀴 · 엔코더/자이로 관측 · 보상 0).

새 환경은 이 폴더를 복사해서 시작하면 된다: envs/<name>/env.py + env_cfg.py.
"""
from __future__ import annotations

from envs.base_env import BaseEnv


class DefaultEnv(BaseEnv):
    pass
