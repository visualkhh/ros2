"""CalibrationEnv — 이 task의 calibration sim 측정 환경 (= DefaultEnv: 순수 sim, 보정 없음). 학습 전 1단계.

측정 장면을 바꾸려면 여기서 (DefaultEnv · BaseEnv 메서드를 덮어쓴다). Isaac 앱이 켜진 뒤 import.
"""
from __future__ import annotations

import torch

from envs.default.env import DefaultEnv


class CalibrationEnv(DefaultEnv):
    def start(self, path=None) -> float:
        """측정 시작 자세: reset → 0.5 s 정지 (스폰 높이에서 떨어져 바퀴 · 캐스터로 서게) → 노란 점 = 목표 경로. 높이 [m] 반환."""
        self.reset()
        zero = torch.zeros(self.num_envs, 2, device=self.device)
        for _ in range(10):
            self.step(zero)
        if path and self.cfg.debug_vis:
            self.set_path(path)
        return float(self.robot.data.root_pos_w.torch[0, 2])
