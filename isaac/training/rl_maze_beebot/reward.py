"""rl_maze_beebot 전용 보상 — rl_maze와 별도로 두고 beebot 것만 바꾼다.

  핵심:
    - 벽에 실제로 부딪히면 큰 패널티(−200) + 에피소드 종료.
    - 그 전에 로봇 몸이 **인플레이션 점군**에 겹치면 작은 패널티(inflate).
      인플레이션 점군은 관측 scan과 같은 `inflation`(0.01~0.05 m)만큼
      장애물 표면에서 로봇 쪽으로 당긴 점이다. 로봇 반경(collide_r=0.18)과
      이 점이 겹치는 경계 = collide_r + inflation.
    - 0.24/0.30 같은 임의의 보상 여유는 두지 않는다.
    - 중간 웨이포인트 보상은 크게(기본 5.0), 최종 goal은 100.
"""
from __future__ import annotations

import math

import torch


def compute_rewards(env) -> torch.Tensor:
    """env(RlMazeBeebotEnv)의 현재 상태 → 보상 [n]. `_get_dones` 뒤에 호출된다."""
    c = env.cfg
    v_enc, w_enc = env.encoders()
    active = (env.grace_steps <= 0).float()

    # 인플레이션 점군과 로봇 몸이 겹친 정도 [0, 1].
    #   wall_clearance = 로봇 중심 → 장애물 표면
    #   인플레이션 점 = 표면에서 로봇 쪽으로 inflation 만큼 당긴 점
    #   로봇 반경 collide_r 이 그 점에 닿는 순간 = collide_r + inflation
    inflate_span = env.inflation.clamp(min=1e-3)
    wall_inflate = ((c.collide_r + env.inflation - env.wall_clearance).clamp(min=0.0)
                    / inflate_span).clamp(0.0, 1.0) * active
    # 주행 특화 — 다음 웨이포인트를 향한 방향 오차 err의 cos: 정면=1 · 옆=0 · 뒤=−1
    xy, yaw = env.pose()
    to_goal = env.goal_point() - xy
    cos_err = torch.cos(torch.atan2(to_goal[..., 1], to_goal[..., 0]) - yaw)
    # 벽 사이 중앙 — 좌우(±90°) 벽까지 거리 l, r. 양쪽 벽이 center_range 안에 다 있을 때(복도)만 |l−r|/(l+r)에 패널티.
    #   한쪽만 벽이거나 열린 곳(모서리 · 갈림길)은 0 — 경로를 따라 도는 것과 부딪히지 않게
    side = env.lidar(xy, yaw, torch.tensor([math.pi / 2, -math.pi / 2], device=xy.device))
    left, right = side[:, 0], side[:, 1]
    corridor = ((left < c.center_range) & (right < c.center_range)).float()
    off_center = (left - right).abs() / (left + right).clamp(min=1e-3) * corridor
    # 경로 이탈 — 지난 웨이포인트 → 지금 웨이포인트 선분까지의 거리. 경로는 벽에서 ≥0.37 m라 경로를 지키면 벽 간격이 따라온다.
    #   모서리를 안쪽으로 질러가면(건너뛰기 규칙이 허용) 커진다. 허용 오차 없음 — 1 cm 벗어나도 −.
    last = env.wp_idx.sub(1).clamp(min=0)
    a, b = env.wps[env._rows, last], env.goal_point()
    ab = b - a
    t = (((xy - a) * ab).sum(-1) / ab.square().sum(-1).clamp(min=1e-6)).clamp(0.0, 1.0)
    off_path = torch.linalg.norm(xy - (a + t[:, None] * ab), dim=-1)
    action_over = env.action_raw - env.action_raw.clamp(-1.0, 1.0)

    terms = {
        "progress": c.w_progress * env.progress.clamp(-0.25, 0.25),
        "reach": c.w_reach * env.reached.float(),
        "goal": c.w_goal * env.goal_done.float(),
        "time": torch.full_like(env.progress, -c.w_time),
        "collision": -c.w_collide * env.collided.float(),
        "inflate": -float(getattr(c, "w_inflate", 0.5)) * wall_inflate,
        "action": -float(getattr(c, "w_action", 0.1)) * action_over.square().sum(-1),
        "smooth": -c.w_smooth * (env.action - env.prev_action).square().sum(-1),
        "speed": c.w_speed * (v_enc / c.max_lin).clamp(0.0, 1.0) * cos_err.clamp(min=0.0) * active,   # 길 쪽을 보고 빨리 달릴수록 +
        "heading": -c.w_heading * (1.0 - cos_err) * 0.5 * active,                                      # 길에서 몸이 틀어질수록 −
        "spin": -c.w_spin * (w_enc / c.max_ang).square() * active,                                     # 제자리 · 지그재그 회전 −
        "center": -c.w_center * off_center * active,                                                   # 복도에서 한쪽 벽에 치우칠수록 −
        "path": -float(getattr(c, "w_path", 0.5)) * off_path.clamp(max=0.5) * active,        # 경로에서 벗어나는 만큼 −, 여유 없음
        "back": -c.w_back * (-v_enc).clamp(min=0.0),
    }
    r = torch.stack(list(terms.values()), dim=0).sum(dim=0)

    log = env.extras.setdefault("log", {})
    for name, value in terms.items():
        log[f"Reward/{name}"] = value.mean().item()
    log["Reward/total"] = r.mean().item()
    log["Metrics/waypoints_done"] = (env.wp_idx - 1).float().mean().item()
    log["Metrics/goal_rate"] = env.goal_done.float().mean().item()
    log["Metrics/collision_rate"] = env.collided.float().mean().item()
    log["Metrics/wall_collision_rate"] = env.wall_hit.float().mean().item()
    log["Metrics/wall_clear"] = env.wall_clearance.mean().item()
    log["Metrics/inflate_overlap"] = wall_inflate.mean().item()
    log["Metrics/off_center"] = off_center.sum().item() / corridor.sum().clamp(min=1.0).item()   # 복도에 있을 때 평균 치우침 (0=중앙)
    log["Metrics/off_path"] = off_path.mean().item()
    log["Metrics/heading_cos"] = cos_err.mean().item()
    log["Metrics/inflation"] = env.inflation.mean().item()
    return r


if __name__ == "__main__":   # 자체 확인 (주행 항 부호) — isaac/에서: PYTHONPATH=. <Isaac venv python> training/rl_maze_beebot/reward.py
    from types import SimpleNamespace as NS

    cfg = NS(collide_r=0.18, w_progress=0, w_reach=0, w_goal=0, w_time=0, w_collide=0, w_smooth=0, w_back=0, w_inflate=0, w_action=0,
             w_speed=0.05, w_heading=0.02, w_spin=0.02, w_center=0.03, w_path=0.5, center_range=1.0, max_lin=0.26, max_ang=1.0)

    def terms(yaw, v, w, lr=(0.5, 0.5), pos=(0.0, 0.0)):
        """목표는 +x 방향 1 m 앞 — 로봇은 yaw로 서서 v · w로 움직임. → (speed, heading, spin)"""
        z, b = torch.zeros(1), torch.zeros(1, dtype=torch.bool)
        env = NS(cfg=cfg, encoders=lambda: (z + v, z + w), grace_steps=z, inflation=z + 0.03, wall_clearance=z + 5,
                 action_raw=torch.zeros(1, 2), action=torch.zeros(1, 2), prev_action=torch.zeros(1, 2), progress=z, reached=b,
                 goal_done=b, collided=b, wall_hit=b, extras={}, pose=lambda: (torch.tensor([pos]), z + yaw), wp_idx=torch.ones(1, dtype=torch.long), _rows=torch.zeros(1, dtype=torch.long),
                 wps=torch.tensor([[[0.0, 0.0], [1.0, 0.0]]]), goal_point=lambda: torch.tensor([[1.0, 0.0]]),
                 lidar=lambda xy, yaw, ang: torch.tensor([lr]))
        compute_rewards(env)
        lg = env.extras["log"]
        return lg["Reward/speed"], lg["Reward/heading"], lg["Reward/spin"], lg["Reward/center"], lg["Reward/path"]

    s0, h0, p0, c0, _ = terms(0.0, 0.26, 0.0)     # 정면으로 최고속
    assert s0 > 0 and h0 == 0 and p0 == 0
    s1, h1, _, _, _ = terms(1.57, 0.26, 0.0)     # 옆을 보고 달림 → 속도 보상 없음 · 방향 패널티
    assert s1 < 1e-3 and h1 < 0
    assert terms(3.14, 0.26, 0.0)[1] < h1  # 뒤돌아 있으면 더 큰 패널티
    assert terms(0.0, 0.26, 1.0)[2] < 0    # 빠르게 돌면 회전 패널티
    assert c0 == 0                                        # 중앙이면 패널티 없음
    assert terms(0.0, 0.0, 0.0, (0.2, 0.8))[3] < 0        # 복도에서 한쪽으로 치우침 → 패널티
    assert terms(0.0, 0.0, 0.0, (0.2, 3.5))[3] == 0       # 한쪽이 열려 있으면(모서리) 0
    assert terms(0.0, 0.0, 0.0, pos=(0.5, 0.0))[4] == 0     # 선 위 → 0
    assert terms(0.0, 0.0, 0.0, pos=(0.5, 0.01))[4] < 0     # 1 cm만 벗어나도 −
    assert terms(0.0, 0.0, 0.0, pos=(0.5, 0.25))[4] < terms(0.0, 0.0, 0.0, pos=(0.5, 0.12))[4] < 0   # 멀수록 더 −
    print("reward self-check ok")
