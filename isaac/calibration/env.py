"""CalibrationEnv — calibration sim 측정 환경 (= DefaultEnv: 순수 sim, 보정 없음). training/<task>/env.py와 같은 자리.

측정 장면을 바꾸려면 여기서 (DefaultEnv · BaseEnv 메서드를 덮어쓴다). Isaac 앱이 켜진 뒤 import.
"""
from __future__ import annotations

from envs.default.env import DefaultEnv


class CalibrationEnv(DefaultEnv):
    pass
