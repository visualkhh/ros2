"""학습 명령 — env마다 0.5~2 s(hold_range)마다 새로 뽑는다: 텔레오퍼 키 (p_teleop) · 부드러운 곡선 (p_smooth) · 정지 (나머지).

텔레오퍼 키 8방향 = tool/cli 키보드 운전과 같은 움직임: 전/후 · 제자리 좌/우 회전 · 사선 4개(반경 DIAG_RADIUS 원호).
로봇 수천 대에 한꺼번에 뽑으므로 torch 텐서 단위 (commands/의 시각 → (v, w) 한 대짜리와 다르다).
"""
from __future__ import annotations

import torch

from isaaclab.utils.math import sample_uniform

from robots.beebot import config as beebot

# 텔레오퍼 키 → (전후, 회전) 방향 (tool/cli/op/bus.py TELEOP_MOVES와 같음)
TELEOP_MOVES = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1))


def sample(k: int, cfg, device) -> tuple[torch.Tensor, torch.Tensor]:
    """명령 k개 → (cmd [k, 2] = [v, w], hold [k] = 유지 시간 s). cfg: p_teleop · p_smooth · hold_range."""
    u = torch.rand(k, device=device)
    speed = sample_uniform(0.10, beebot.MAX_LIN, (k,), device)
    turn = sample_uniform(0.40, beebot.MAX_ANG, (k,), device)
    moves = torch.tensor(TELEOP_MOVES, dtype=torch.float32, device=device)[torch.randint(0, len(TELEOP_MOVES), (k,), device=device)]
    # 텔레오퍼: 사선은 설정 속도 + 반경 DIAG_RADIUS 원호, 제자리 회전은 회전 속도
    v = moves[:, 0] * speed
    w = torch.where(moves[:, 0] != 0, moves[:, 1] * speed / beebot.DIAG_RADIUS, moves[:, 1] * turn)
    smooth = (u >= cfg.p_teleop) & (u < cfg.p_teleop + cfg.p_smooth)
    v = torch.where(smooth, sample_uniform(0.0, beebot.MAX_LIN, (k,), device), v)
    w = torch.where(smooth, sample_uniform(-1.0, 1.0, (k,), device), w)
    stop = u >= cfg.p_teleop + cfg.p_smooth
    v, w = v * (~stop), w * (~stop)
    return torch.stack([v, w], dim=-1), sample_uniform(*cfg.hold_range, (k,), device)
