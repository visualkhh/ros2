"""RlCorrectionWaypointCommand — 학습 명령: env마다 랜덤 웨이포인트 num_waypoints개 (시작 자세 기준 좌표).

시작 (0, 0) · 방향 0에서 출발 → 점마다 앞 방향에서 turn만큼 꺾고 dist만큼 간 곳. 로봇 수천 대분을 한꺼번에 (torch).
실제 로봇에서는 같은 좌표를 /rl_goal(goal_mode=absolute)로 하나씩 보낸다.
"""
from __future__ import annotations

import torch

from isaaclab.utils.math import sample_uniform


class RlCorrectionWaypointCommand:
    def __init__(self, num: int = 4, dist: tuple[float, float] = (0.5, 1.5), turn: tuple[float, float] = (-3.14159, 3.14159)):
        self.num, self.dist, self.turn = num, dist, turn

    def sample(self, k: int, device) -> torch.Tensor:
        """웨이포인트 [k, num, 2] (x 앞, y 왼쪽 — 시작 자세 기준)."""
        d = sample_uniform(*self.dist, (k, self.num), device)
        heading = torch.cumsum(sample_uniform(*self.turn, (k, self.num), device), dim=1)
        steps = torch.stack([d * torch.cos(heading), d * torch.sin(heading)], dim=-1)
        return torch.cumsum(steps, dim=1)


if __name__ == "__main__":   # 자체 확인 — Isaac venv: isaaclab.sh -p training/rl_correction_waypoint/command.py
    w = RlCorrectionWaypointCommand().sample(1000, "cpu")
    seg = torch.diff(torch.cat([torch.zeros(1000, 1, 2), w], dim=1), dim=1).norm(dim=-1)
    assert w.shape == (1000, 4, 2) and (seg >= 0.5 - 1e-5).all() and (seg <= 1.5 + 1e-5).all()
    print("ok — 시작에서 가장 먼 점", round(float(w.norm(dim=-1).max()), 2), "m")
