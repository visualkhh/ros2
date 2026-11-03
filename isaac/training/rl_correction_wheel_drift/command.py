"""TeleopCommand — 학습 명령: hold_range(0.5~2 s)마다 새로 뽑는다 — 텔레오퍼 키 (p_teleop) · 부드러운 곡선 (p_smooth) · 정지 (나머지).

텔레오퍼 키 8방향 = tool/cli 키보드 운전과 같은 움직임: 전/후 · 제자리 좌/우 회전 · 사선 4개(반경 DIAG_RADIUS 원호).
  sample(k, device)  학습 env용 — 로봇 k대 분량을 한꺼번에 (torch)
  __call__(t)        BaseCommand 약속 — 한 대용, 유지 시간이 지나면 같은 분포에서 새로
                     → ./run-world.sh --cmd training.rl_correction_wheel_drift.command:TeleopCommand (학습 명령을 눈으로)
"""
from __future__ import annotations

import torch

from isaaclab.utils.math import sample_uniform

from commands.base_command import BaseCommand
from robots.beebot import config as beebot

# 텔레오퍼 키 → (전후, 회전) 방향 (tool/cli/op/bus.py TELEOP_MOVES와 같음)
TELEOP_MOVES = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1))


class TeleopCommand(BaseCommand):
    """speed · turn = 상한 (전진 0.10~speed · 제자리 회전 0.40~turn). 학습은 로봇 최대값(MAX_LIN · MAX_ANG)."""

    def __init__(self, speed: float = beebot.MAX_LIN, turn: float = beebot.MAX_ANG,
                 p_teleop: float = 0.6, p_smooth: float = 0.3, hold_range: tuple[float, float] = (0.5, 2.0)):
        super().__init__(speed, turn)
        self.p_teleop, self.p_smooth, self.hold_range = p_teleop, p_smooth, hold_range
        self.reset()

    def sample(self, k: int, device) -> tuple[torch.Tensor, torch.Tensor]:
        """명령 k개 → (cmd [k, 2] = [v, w], hold [k] = 유지 시간 s)."""
        u = torch.rand(k, device=device)
        speed = sample_uniform(min(0.10, self.speed), self.speed, (k,), device)
        turn = sample_uniform(min(0.40, self.turn), self.turn, (k,), device)
        moves = torch.tensor(TELEOP_MOVES, dtype=torch.float32, device=device)[torch.randint(0, len(TELEOP_MOVES), (k,), device=device)]
        # 텔레오퍼: 사선은 설정 속도 + 반경 DIAG_RADIUS 원호, 제자리 회전은 회전 속도
        v = moves[:, 0] * speed
        w = torch.where(moves[:, 0] != 0, moves[:, 1] * speed / beebot.DIAG_RADIUS, moves[:, 1] * turn)
        smooth = (u >= self.p_teleop) & (u < self.p_teleop + self.p_smooth)
        v = torch.where(smooth, sample_uniform(0.0, self.speed, (k,), device), v)
        w = torch.where(smooth, sample_uniform(-1.0, 1.0, (k,), device), w)
        stop = u >= self.p_teleop + self.p_smooth
        v, w = v * (~stop), w * (~stop)
        return torch.stack([v, w], dim=-1), sample_uniform(*self.hold_range, (k,), device)

    def reset(self) -> None:
        self._until, self._cmd = 0.0, (0.0, 0.0)

    def __call__(self, t: float) -> tuple[float, float]:
        if t >= self._until:
            cmd, hold = self.sample(1, "cpu")
            self._cmd, self._until = tuple(cmd[0].tolist()), t + float(hold[0])
        return self._cmd
