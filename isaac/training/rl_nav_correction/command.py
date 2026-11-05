"""RlNavCorrectionCommand — 학습 명령: env마다 [v, w]가 0.5~2 s마다 바뀐다 (정지 · 직진 · 제자리 회전 · 호).

nav2 컨트롤러가 /cmd_vel_nav로 내는 명령과 같은 모양 (한 값을 잠시 유지). 로봇 수천 대분을 한꺼번에 (torch).
"""
from __future__ import annotations

import torch

from isaaclab.utils.math import sample_uniform


class RlNavCorrectionCommand:
    def __init__(self, max_lin: float, max_ang: float, hold=(0.5, 2.0), stop_prob=0.15, straight_prob=0.30, turn_prob=0.20,
                 lin_frac=(0.1, 0.8), ang_frac=(0.1, 0.6), reverse_prob=0.1):
        self.max_lin, self.max_ang, self.hold = max_lin, max_ang, hold
        self.cut = (stop_prob, stop_prob + straight_prob, stop_prob + straight_prob + turn_prob)   # 나머지 = 호
        self.lin_frac, self.ang_frac, self.reverse_prob = lin_frac, ang_frac, reverse_prob

    def sample(self, k: int, device) -> torch.Tensor:
        """명령 [k, 2] = (v, w)."""
        sign = lambda p: torch.where(torch.rand(k, device=device) < p, -1.0, 1.0)  # noqa: E731
        v = sample_uniform(*self.lin_frac, (k,), device) * self.max_lin * sign(self.reverse_prob)
        w = sample_uniform(*self.ang_frac, (k,), device) * self.max_ang * sign(0.5)
        m = torch.rand(k, device=device)
        keep_v = (m >= self.cut[0]) & ((m < self.cut[1]) | (m >= self.cut[2]))   # 직진 · 호
        keep_w = m >= self.cut[1]                                                  # 회전 · 호
        return torch.stack([v * keep_v, w * keep_w], dim=-1)

    def hold_steps(self, k: int, dt: float, device) -> torch.Tensor:
        """다음 명령 유지 스텝 수 [k] (최소 1)."""
        return (sample_uniform(*self.hold, (k,), device) / dt).round().long().clamp(min=1)


if __name__ == "__main__":   # 자체 확인 — Isaac venv: isaaclab.sh -p training/rl_nav_correction/command.py
    c = RlNavCorrectionCommand(0.26, 1.82)
    x = c.sample(20000, "cpu")
    stop = ((x[:, 0] == 0) & (x[:, 1] == 0)).float().mean()
    turn = ((x[:, 0] == 0) & (x[:, 1] != 0)).float().mean()
    assert x[:, 0].abs().max() <= 0.26 * 0.8 + 1e-6 and x[:, 1].abs().max() <= 1.82 * 0.6 + 1e-6
    assert abs(stop - 0.15) < 0.02 and abs(turn - 0.20) < 0.02, (stop, turn)
    h = c.hold_steps(1000, 0.05, "cpu")
    assert h.min() >= 10 and h.max() <= 40
    print("ok — 정지", round(float(stop), 2), "회전", round(float(turn), 2))
