"""rl_maze_beebot 전용 보상 — rl_maze와 별도로 두고 beebot 것만 바꾼다.

  핵심:
    - 벽/사람/공에 실제로 부딪히면 큰 패널티(−200) + 에피소드 종료.
    - 그 전에 로봇 몸이 **인플레이션 점군**에 겹치면 작은 패널티(inflate).
      인플레이션 점군은 관측 scan과 같은 `inflation`(0.01~0.05 m)만큼
      장애물 표면에서 로봇 쪽으로 당긴 점이다. 로봇 반경(collide_r=0.18)과
      이 점이 겹치는 경계 = collide_r + inflation.
    - 0.24/0.30 같은 임의의 보상 여유는 두지 않는다.
    - 중간 웨이포인트 보상은 크게(기본 5.0), 최종 goal은 100.
    - 사람·공 회피: 가까워지면 감점 · 멀어지면 가점인 potential 차이 (avoid). 충돌 −N만으로는 초반 정책이 피할 방향을 못 배운다.
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
    action_over = env.action_raw - env.action_raw.clamp(-1.0, 1.0)

    # 사람·공 회피 — potential φ = −((near − d)/(near − collide_r))²  (d = 로봇 중심 → 가장 가까운 사람/공 표면, 접촉 = collide_r)
    #   보상 = w_avoid × (γ·φ_t − φ_{t−1}): 접근하면 −, 멀어지면 +, 가만히 있으면 거의 0 (제자리에 서 있어서 얻는 이득 없음 — potential-based라
    #   최적 정책은 안 바뀌고 학습만 빨라진다). 사람·공이 없으면 d = 1e6이라 0.
    phi = -(((c.avoid_near - env.dyn_clearance) / (c.avoid_near - c.collide_r)).clamp(0.0, 1.0)).square()
    prev_phi = torch.where(env.episode_length_buf <= 1, phi, getattr(env, "_dyn_phi", phi))   # 리셋 직후엔 이전 값 없음
    env._dyn_phi = phi
    avoid = c.w_avoid * (0.99 * phi - prev_phi) * active

    terms = {
        "progress": c.w_progress * env.progress.clamp(-0.25, 0.25),
        "reach": c.w_reach * env.reached.float(),
        "goal": c.w_goal * env.goal_done.float(),
        "time": torch.full_like(env.progress, -c.w_time),
        "collision": -c.w_collide * env.collided.float(),
        "inflate": -float(getattr(c, "w_inflate", 0.5)) * wall_inflate,
        "dyn_inflate": -float(getattr(c, "w_dynamic_inflate", 0.5)) * dyn_inflate,
        "avoid": avoid,
        "action": -float(getattr(c, "w_action", 0.1)) * action_over.square().sum(-1),
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


if __name__ == "__main__":   # 자체 확인 (회피 항의 부호) — isaac/에서: PYTHONPATH=. <Isaac venv python> training/rl_maze_beebot/reward.py
    from types import SimpleNamespace as NS

    cfg = NS(avoid_near=1.0, collide_r=0.18, w_avoid=5.0, w_progress=0, w_reach=0, w_goal=0, w_time=0, w_collide=0, w_smooth=0, w_back=0,
             w_inflate=0, w_dynamic_inflate=0, w_action=0)

    def avoid(prev_d, d, first=False):
        """프레임 두 개를 실제 compute_rewards에 넣어 Reward/avoid를 읽는다."""
        z, b = torch.zeros(1), torch.zeros(1, dtype=torch.bool)
        env = NS(cfg=cfg, encoders=lambda: (z, z), grace_steps=z, inflation=z + 0.03, wall_clearance=z + 5, action_raw=torch.zeros(1, 2),
                 action=torch.zeros(1, 2), prev_action=torch.zeros(1, 2), progress=z, reached=b, goal_done=b, collided=b, wall_hit=b, dyn_hit=b,
                 wp_idx=z, extras={}, episode_length_buf=torch.tensor([10]), dyn_clearance=torch.tensor([prev_d]))
        compute_rewards(env)                                   # 이전 프레임 (φ 저장)
        env.dyn_clearance, env.episode_length_buf = torch.tensor([d]), torch.tensor([1 if first else 11])
        compute_rewards(env)
        return env.extras["log"]["Reward/avoid"]

    assert avoid(1.00, 0.90) < 0                 # 접근 = 감점
    assert avoid(0.60, 0.70) > 0                 # 멀어짐 = 가점
    assert avoid(5.0, 5.0) == 0                  # 멀리 있으면 0
    assert abs(avoid(0.5, 0.5)) < 0.06           # 같은 거리 유지 = 거의 0 (γ 때문에 아주 조금)
    assert abs(avoid(1e6, 0.5, first=True)) < 0.06   # 리셋 직후엔 이전 값 없음
    print("ok — avoid 항: 접근 −, 이탈 +, 먼 곳 0")
