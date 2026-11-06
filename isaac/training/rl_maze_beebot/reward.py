"""rl_maze_beebot 전용 보상 — rl_maze의 보상과 내용이 겹치더라도 beebot 것만 따로 바꾼다.

  rl_maze의 `_get_rewards`를 복사해 왔지만, beebot은 observation inflation(0.005~0.02 m)과
  보상용 여유(`reward_margin`)를 분리한다. path_margin=0.32로 경로는 벽에서 0.34 m를 확보하므로
  보상도 그 근처(기본 0.30 m)부터 soft 패널티를 주어야 의미가 있다.
"""
from __future__ import annotations

import torch


def compute_rewards(env) -> torch.Tensor:
    """env(RlMazeBeebotEnv)의 현재 상태 → 보상 [n]. `_get_dones` 뒤에 호출된다."""
    c = env.cfg
    v_enc, _ = env.encoders()
    active = (env.grace_steps <= 0).float()

    # 보상용 여유 — 관측용 inflation(0.005~0.02)과 별개.
    wall_margin = float(getattr(c, "reward_margin", c.inflation_range[1]))
    dyn_margin = float(getattr(c, "reward_dynamic_margin", wall_margin))
    wall_span = max(wall_margin - c.collide_r, 1e-3)
    dyn_span = max(dyn_margin - c.collide_r, 1e-3)
    wall_overlap = (wall_margin - env.wall_clearance).clamp(min=0.0)
    dyn_overlap = (dyn_margin - env.dyn_clearance).clamp(min=0.0)
    wall_near = (wall_overlap / wall_span).clamp(0.0, 1.0) * active
    dyn_near = (dyn_overlap / dyn_span).clamp(0.0, 1.0) * active

    terms = {
        "progress": c.w_progress * env.progress.clamp(-0.25, 0.25),
        "reach": c.w_reach * env.reached.float(),
        "goal": c.w_goal * env.goal_done.float(),
        "time": torch.full_like(env.progress, -c.w_time),
        "collision": -c.w_collide * env.collided.float(),
        "wall": -c.w_wall * wall_near,
        "dynamic": -c.w_dynamic * dyn_near,
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
    log["Metrics/wall_overlap"] = wall_overlap.mean().item()
    log["Metrics/dyn_overlap"] = dyn_overlap.mean().item()
    log["Metrics/inflation"] = env.inflation.mean().item()
    log["Metrics/reward_margin"] = wall_margin
    log["Metrics/reward_dynamic_margin"] = dyn_margin
    return r
