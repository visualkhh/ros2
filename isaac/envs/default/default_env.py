"""DefaultEnv — 아무 규칙 없는 기본 세상. BaseEnv가 다 해 준다 ([v, w] → 드라이버 식 → 바퀴 · 엔코더/자이로 관측 · 보상 0).

새 환경은 이걸 복사해서 시작하면 된다: <name>_env.py + <name>_env_cfg.py (Isaac Lab task 폴더 관례).
"""
from __future__ import annotations

from envs.base_env import BaseEnv


class DefaultEnv(BaseEnv):
    pass
