"""RlMazeCommand — 학습 명령: 미로 안 랜덤 경로(웨이포인트 묶음, maze_grid.py가 A*로 미리 뽑은 paths.npz)에서 env마다 하나씩.

nav2라면 planner가 주는 경로 자리 — 정책(controller)은 이 웨이포인트를 따라 가장 빨리 도착하는 걸 배운다. 좌표 = 미로 기준 (env에서 미로 = 월드 원점).
"""
from __future__ import annotations

import numpy as np
import torch

from commands.rl_base_command import RlBaseCommand
from training.rl_maze.maze_grid import GRID_NPZ, PATHS_NPZ, load


class RlMazeCommand(RlBaseCommand):
    START_CLEAR = 0.32   # [m] 출발점은 벽에서 이만큼 떨어진 경로만 (충돌 0.17 + 여유 — 랜덤 정책도 출발 직후 벽에 안 박게)
    MIN_CLEAR = 0.30     # [m] 경로 전체가 벽에서 이만큼은 떨어진 것만 (inflation 최대값 + 여유)

    def __init__(self, path: str = PATHS_NPZ, min_clear: float | None = None, start_clear: float | None = None):
        d = np.load(path)
        pts, lens = d["pts"], d["lens"]    # [N, W, 2] (남는 자리 = 마지막 점) · [N] 실제 점 개수 (첫 점 = 출발)
        g = load(GRID_NPZ)
        x0, y0, res = g["origin"][0], g["origin"][1], g["res"]
        ix = np.clip(((pts[..., 0] - x0) / res).astype(int), 0, g["occ"].shape[1] - 1)
        iy = np.clip(((pts[..., 1] - y0) / res).astype(int), 0, g["occ"].shape[0] - 1)
        clear = g["dist"][iy, ix]          # [N, W] — 경로 점마다 벽까지 거리
        min_clear = self.MIN_CLEAR if min_clear is None else float(min_clear)
        start_clear = self.START_CLEAR if start_clear is None else float(start_clear)
        keep = (clear[:, 0] >= start_clear) & (clear.min(axis=1) >= min_clear)
        self.pts = torch.as_tensor(pts[keep])
        self.lens = torch.as_tensor(lens[keep])

    def sample(self, k: int, device) -> tuple[torch.Tensor, torch.Tensor]:
        """경로 k개 → (웨이포인트 [k, W, 2], 개수 [k])."""
        if self.lens.device != torch.device(device):
            self.pts, self.lens = self.pts.to(device), self.lens.to(device)
        i = torch.randint(0, len(self.lens), (k,), device=self.lens.device)
        return self.pts[i], self.lens[i]
