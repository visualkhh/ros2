"""RlMazeCommand — 학습 명령: 미로 안 랜덤 경로(웨이포인트 묶음, maze_grid.py가 A*로 미리 뽑은 paths.npz)에서 env마다 하나씩.

nav2라면 planner가 주는 경로 자리 — 정책(controller)은 이 웨이포인트를 따라 가장 빨리 도착하는 걸 배운다. 좌표 = 미로 기준 (env에서 미로 = 월드 원점).
"""
from __future__ import annotations

import numpy as np
import torch

from commands.rl_base_command import RlBaseCommand
from training.rl_maze.maze_grid import PATHS_NPZ


class RlMazeCommand(RlBaseCommand):
    def __init__(self, path: str = PATHS_NPZ):
        d = np.load(path)
        self.pts = torch.as_tensor(d["pts"])    # [N, W, 2] (남는 자리 = 마지막 점) — sample의 device로 처음 쓸 때 옮김
        self.lens = torch.as_tensor(d["lens"])  # [N] 실제 점 개수 (첫 점 = 출발)

    def sample(self, k: int, device) -> tuple[torch.Tensor, torch.Tensor]:
        """경로 k개 → (웨이포인트 [k, W, 2], 개수 [k])."""
        if self.lens.device != torch.device(device):
            self.pts, self.lens = self.pts.to(device), self.lens.to(device)
        i = torch.randint(0, len(self.lens), (k,), device=self.lens.device)
        return self.pts[i], self.lens[i]
