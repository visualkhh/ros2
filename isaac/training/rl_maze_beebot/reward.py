"""rl_maze_beebot 전용 보상 — rl_maze와 별도로 두고 beebot 것만 바꾼다.

  핵심:
    - 벽/사람/공에 실제로 부딪히면 큰 패널티(−200) + 에피소드 종료.
    - 그 전에 로봇 몸이 **인플레이션 점군**에 겹치면 작은 패널티(inflate).
      인플레이션 점군은 관측 scan과 같은 `inflation`(0.01~0.05 m)만큼
      장애물 표면에서 로봇 쪽으로 당긴 점이다. 로봇 반경(collide_r=0.18)과
      이 점이 겹치는 경계 = collide_r + inflation.
    - 0.24/0.30 같은 임의의 보상 여유는 두지 않는다.
    - 중간 웨이포인트 보상은 크게(기본 5.0), 최종 goal은 100.
"""
from __future__ import annotations

import torch


def compute_rewards(env) -> torch.Tensor:
    """env(RlMazeBeebotEnv)의 현재 상태 → 보상 [n]. `_get_dones` 뒤에 호출된다."""
    c = env.cfg
    v_enc, _ = env.encoders()
    active = (env.grace_steps <= 0).float()

    # 인플레이션 점군과 로봇 몸이 겹친 정도 [0, 1].
    #   wall_clearance = 로봇 중심 → 장애물 표면
    #   인플레이션 점 = 표면에서 로봇 쪽으로 inflation 만큼 당긴 점
    #   로봇 반경 collide_r 이 그 점에 닿는 순간 = collide_r + inflation
    inflate_span = env.inflation.clamp(min=1e-3)
    wall_inflate = ((c.collide_r + env.inflation - env.wall_clearance).clamp(min=0.0)
                    / inflate_span).clamp(0.0, 1.0) * active
    dyn_inflate = ((c.collide_r + env.inflation - env.dyn_clearance).clamp(min=0.0)
                   / inflate_span).clamp(0.0, 1.0) * active

    terms = {
        "progress": c.w_progress * env.progress.clamp(-0.25, 0.25),
        "reach": c.w_reach * env.reached.float(),
        "goal": c.w_goal * env.goal_done.float(),
        "time": torch.full_like(env.progress, -c.w_time),
        "collision": -c.w_collide * env.collided.float(),
        "inflate": -float(getattr(c, "w_inflate", 0.5)) * wall_inflate,
        "dyn_inflate": -float(getattr(c, "w_dynamic_inflate", 0.5)) * dyn_inflate,
        "smooth": -c.w_smooth * (env.action - env.prev_action).square().sum(-1),
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
    log["Metrics/dyn_collision_rate"] = env.dyn_hit.float().mean().item()
    log["Metrics/wall_clear"] = env.wall_clearance.mean().item()
    log["Metrics/dyn_clear"] = env.dyn_clearance.clamp(max=2.0).mean().item()
    log["Metrics/inflate_overlap"] = wall_inflate.mean().item()
    log["Metrics/dyn_inflate_overlap"] = dyn_inflate.mean().item()
    log["Metrics/inflation"] = env.inflation.mean().item()
    return r
