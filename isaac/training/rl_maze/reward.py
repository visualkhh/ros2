"""rl_maze 보상 — 진행(웨이포인트 따라가기)이 주 신호, 나머지는 작은 모양 잡기. rl_maze_beebot/reward.py와 같은 구조.

  크기 (한 스텝, 20 Hz, 최고속 0.26 m/s): progress ≈ +0.13 · speed ≤ +0.05 · time −0.02 ·
    path/inflate/center/heading/spin은 각각 몇 cm~수십 cm 벗어나거나 가까울 때 −0.0x ~ −0.1 대.
  한 번만 나오는 값: 충돌 −20 (에피소드 끝) · 도착 +20 · 웨이포인트 통과 +1.
  벽 근접(inflate): 로봇 몸(collide_r)이 인플레이션 점군에 겹치는 순간부터 [0, 1]로 커지는 soft 벌점.
    경계 = collide_r + inflation(0.01~0.05 m) — 표면 1~5 cm 안에서만 켜진다 (벽 간격은 path 항과 경로가 맡는다).
  path: 지난 → 지금 웨이포인트 선분까지의 거리 (허용 오차 없음). 경로가 벽에서 떨어져 있으니 지키면 벽 간격이 따라온다.
"""
from __future__ import annotations

import math

import torch


def compute_rewards(env) -> torch.Tensor:
    """env(RlMazeEnv)의 현재 상태 → 보상 [n]. `_get_dones` 뒤에 호출된다."""
    c = env.cfg
    v_enc, w_enc = env.encoders(env.r_nom, env.sep_nom)
    active = (env.grace_steps <= 0).float()

    # 인플레이션 점군과 로봇 몸이 겹친 정도 [0, 1] — clearance = 로봇 중심 → 표면. 몸이 점에 닿는 순간 = collide_r + inflation
    span = env.inflation.clamp(min=1e-3)

    def near(clearance):
        return ((c.collide_r + env.inflation - clearance).clamp(min=0.0) / span).clamp(0.0, 1.0) * active

    wall_inflate = near(env.wall_clearance)

    xy, yaw = env.pose()
    to_goal = env.goal_point() - xy
    cos_err = torch.cos(torch.atan2(to_goal[..., 1], to_goal[..., 0]) - yaw)   # 정면=1 · 옆=0 · 뒤=−1
    # 복도(좌우 ±90° 벽이 둘 다 center_range 안)에서만 |l−r|/(l+r) — 한쪽이 열린 곳(모서리)은 0
    side = env.lidar(xy, yaw, torch.tensor([math.pi / 2, -math.pi / 2], device=xy.device))
    left, right = side[:, 0], side[:, 1]
    corridor = ((left < c.center_range) & (right < c.center_range)).float()
    off_center = (left - right).abs() / (left + right).clamp(min=1e-3) * corridor
    # 경로 이탈 — 지난 웨이포인트 → 지금 웨이포인트 선분까지의 거리
    a, b = env.wps[env._rows, env.wp_idx.sub(1).clamp(min=0)], env.goal_point()
    ab = b - a
    t = (((xy - a) * ab).sum(-1) / ab.square().sum(-1).clamp(min=1e-6)).clamp(0.0, 1.0)
    off_path = torch.linalg.norm(xy - (a + t[:, None] * ab), dim=-1)
    action_over = env.action_raw - env.action_raw.clamp(-1.0, 1.0)   # 정책 원출력이 [-1, 1]을 넘은 만큼

    terms = {
        "progress": c.w_progress * env.progress.clamp(-0.25, 0.25),
        "reach": c.w_reach * env.reached.float(),
        "goal": c.w_goal * env.goal_done.float(),
        "time": torch.full_like(env.progress, -c.w_time),
        "collision": -c.w_collide * env.collided.float(),
        "inflate": -c.w_inflate * wall_inflate,
        "action": -c.w_action * action_over.square().sum(-1),
        "smooth": -c.w_smooth * (env.action - env.prev_action).square().sum(-1),
        "speed": c.w_speed * (v_enc / c.max_lin).clamp(0.0, 1.0) * cos_err.clamp(min=0.0) * active,   # 길 쪽을 보고 빨리 달릴수록 +
        "heading": -c.w_heading * (1.0 - cos_err) * 0.5 * active,                                      # 길에서 몸이 틀어질수록 −
        "spin": -c.w_spin * (w_enc / c.max_ang).square() * active,                                     # 제자리 · 지그재그 회전 −
        "center": -c.w_center * off_center * active,                                                   # 복도에서 한쪽 벽에 치우칠수록 −
        "path": -c.w_path * off_path.clamp(max=0.5) * active,                                          # 경로에서 벗어날수록 −
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


if __name__ == "__main__":   # 자체 확인 — isaac/에서: PYTHONPATH=. <Isaac venv python> training/rl_maze/reward.py
    from types import SimpleNamespace as NS

    z0 = dict(w_progress=0, w_reach=0, w_goal=0, w_time=0, w_collide=0, w_smooth=0, w_back=0, w_action=0)
    cfg = NS(collide_r=0.17, w_inflate=1.0, w_speed=0.05, w_heading=0.02, w_spin=0.02, w_center=0.03, w_path=0.5,
             center_range=1.0, max_lin=0.26, max_ang=1.82, **z0)

    def terms(yaw=0.0, v=0.26, w=0.0, lr=(0.5, 0.5), pos=(0.0, 0.0), wall=5.0):
        """목표는 +x 1 m 앞, 경로는 (0,0)→(1,0). → 항 이름: 값"""
        z, b = torch.zeros(1), torch.zeros(1, dtype=torch.bool)
        env = NS(cfg=cfg, encoders=lambda *a: (z + v, z + w), r_nom=None, sep_nom=None, grace_steps=z, inflation=z + 0.03,
                 wall_clearance=z + wall, action_raw=torch.zeros(1, 2), action=torch.zeros(1, 2),
                 prev_action=torch.zeros(1, 2), progress=z, reached=b, goal_done=b, collided=b, wall_hit=b, wp_idx=torch.ones(1, dtype=torch.long),
                 _rows=torch.zeros(1, dtype=torch.long), wps=torch.tensor([[[0.0, 0.0], [1.0, 0.0]]]), extras={},
                 pose=lambda: (torch.tensor([pos]), z + yaw), goal_point=lambda: torch.tensor([[1.0, 0.0]]),
                 lidar=lambda xy, yaw, ang: torch.tensor([lr]))
        compute_rewards(env)
        return {k[7:]: v for k, v in env.extras["log"].items() if k.startswith("Reward/")}

    base = terms()
    assert base["speed"] > 0 and base["heading"] == 0 and base["spin"] == 0 and base["center"] == 0 and base["path"] == 0
    assert terms(yaw=1.57)["speed"] < 1e-3 and terms(yaw=1.57)["heading"] < 0
    assert terms(yaw=3.14)["heading"] < terms(yaw=1.57)["heading"]       # 뒤돌아 있으면 더 큰 벌점
    assert terms(w=1.0)["spin"] < 0
    assert terms(v=0, lr=(0.2, 0.8))["center"] < 0 and terms(v=0, lr=(0.2, 3.5))["center"] == 0   # 복도만 · 열린 곳 0
    assert terms(pos=(0.5, 0.25))["path"] < terms(pos=(0.5, 0.12))["path"] < terms(pos=(0.5, 0.01))["path"] < 0
    assert terms(wall=5.0)["inflate"] == 0 and terms(wall=0.19)["inflate"] < 0     # 인플레이션(0.03+0.17) 안 · 밖
    assert terms(wall=0.17)["inflate"] < terms(wall=0.19)["inflate"]               # 가까울수록 더 −
    print("rl_maze reward self-check ok")
