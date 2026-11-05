"""RlBaseCommand — 학습 명령의 바탕. 수천 대분을 한꺼번에 뽑는다 (torch). 표준 라이브러리만 import (torch는 타입 표기뿐).

BaseCommand(시각 t → (v, w), 로봇 한 대 — 칼리브레이션 · 직접 운전)와는 다른 종류다. 학습 task의 command.py가 이걸 상속한다:
  training/<task>/command.py   Rl<Task>Command(RlBaseCommand) — sample(k, device)가 env k개분 명령을 새로 뽑는다 (리셋 · 명령 만료 때)
생성자 인자는 task마다 다르다 (그 task의 cfg 값). 뽑는 모양도 task마다 다르다:
  rl_nav_correction      명령 (v, w) [k, 2] (+ hold_steps 유지 시간)
  rl_correction_waypoint 웨이포인트 [k, num, 2]
  rl_maze                (웨이포인트 [k, W, 2], 점 개수 [k])
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import torch


class RlBaseCommand(ABC):
    @abstractmethod
    def sample(self, k: int, device: "torch.device | str") -> "torch.Tensor | tuple[torch.Tensor, ...]":
        """env k개분 명령을 새로 뽑는다 — device 위의 텐서 (또는 텐서 묶음)."""
