"""명령 · 실제 이동 · 목표 경로를 화면에 그린다 (창·스냅샷 볼 때만).

- 초록 막대 = 명령 (v, w)대로라면 갈 방향 · 파랑 막대 = 실제 몸체 속도로 갈 방향
  (길이 ∝ 속도, 방향 = 지금 방향 + w × 0.25 s — 원호의 현. 제자리 회전도 보이게 최소 길이)
- 노란 점 = 목표 경로 (학습: 명령이 바뀐 순간부터 그 명령대로 갈 길 · calibration: 사각형 전체)
  로봇이 노란 점에서 벗어나는 만큼이 드리프트다.

Isaac Lab VisualizationMarkers를 쓴다 (센서 debug_vis와 같은 기능). 화살표 USD(Nucleus) 대신
막대(Cuboid)·점(Sphere)이라 오프라인에서도 된다. 이 task의 env(학습) · calibration/env.py(측정 · run-world)가 debug_vis_setup · debug_vis_draw에서 쓴다.
"""
from __future__ import annotations

import math

import torch


class CommandMarkers:
    """update()를 부를 때마다 막대·점을 다시 그린다. 입력은 (n,) / (n, 3) 텐서."""

    HORIZON = 0.5        # [s] 막대 방향을 잡을 앞 시간
    LEN_PER_MPS = 3.0    # 막대 길이 [m] / 속도 [m/s]
    MIN_LEN = 0.12       # [m] 제자리 회전도 보이게
    Z_CMD, Z_ACT, Z_PATH = 0.22, 0.27, 0.01   # [m] 로봇 위 높이 (점은 바닥 바로 위)

    def __init__(self, prim_path: str = "/Visuals/CommandMarkers"):
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def bar(rgb):
            return sim_utils.CuboidCfg(size=(1.0, 0.025, 0.025),
                                       visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=rgb))

        self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path=prim_path, markers={
            "cmd": bar((0.1, 0.8, 0.1)),
            "act": bar((0.1, 0.35, 1.0)),
            "path": sim_utils.SphereCfg(radius=0.012,
                                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.8, 0.0))),
        }))

    def set_visible(self, on: bool) -> None:
        self.markers.set_visibility(on)

    @staticmethod
    def yaw_of(quat_w: torch.Tensor) -> torch.Tensor:
        from isaaclab.utils.math import euler_xyz_from_quat   # Lab의 쿼터니언 순서를 따른다
        return euler_xyz_from_quat(quat_w)[2]

    def _bars(self, pos, yaw, v, w, z):
        from isaaclab.utils.math import quat_from_euler_xyz
        heading = yaw + w * self.HORIZON / 2 + math.pi * (v < 0)
        moving = ((v.abs() + w.abs()) > 1e-3).float()
        length = torch.clamp(v.abs() * self.LEN_PER_MPS, min=self.MIN_LEN) * moving
        ctr = pos.clone()
        ctr[:, 0] += torch.cos(heading) * length / 2
        ctr[:, 1] += torch.sin(heading) * length / 2
        ctr[:, 2] += z
        zero = torch.zeros_like(heading)
        scale = torch.stack([length.clamp(min=1e-4), moving.clamp(min=1e-4), moving.clamp(min=1e-4)], dim=-1)
        return ctr, quat_from_euler_xyz(zero, zero, heading), scale

    def update(self, pos_w, yaw, cmd_v, cmd_w, v, w, path_w: torch.Tensor | None = None) -> None:
        """pos_w (n,3) · yaw (n,) · 명령/실제 v, w (n,) · path_w (m,3) 목표 경로 점 (없으면 안 그림)."""
        from isaaclab.utils.math import quat_from_euler_xyz
        n, dev = pos_w.shape[0], pos_w.device
        c, a = self._bars(pos_w, yaw, cmd_v, cmd_w, self.Z_CMD), self._bars(pos_w, yaw, v, w, self.Z_ACT)
        trans, quat, scale = [c[0], a[0]], [c[1], a[1]], [c[2], a[2]]
        idx = [torch.zeros(n, dtype=torch.long, device=dev), torch.ones(n, dtype=torch.long, device=dev)]
        if path_w is not None and len(path_w):
            m = path_w.shape[0]
            zero = torch.zeros(m, device=dev)
            trans.append(path_w)
            quat.append(quat_from_euler_xyz(zero, zero, zero))
            scale.append(torch.ones(m, 3, device=dev))
            idx.append(torch.full((m,), 2, dtype=torch.long, device=dev))
        self.markers.visualize(translations=torch.cat(trans), orientations=torch.cat(quat),
                               scales=torch.cat(scale), marker_indices=torch.cat(idx))

    @staticmethod
    def arc(start: torch.Tensor, v: torch.Tensor, w: torch.Tensor, duration: torch.Tensor,
            points: int = 12) -> torch.Tensor:
        """start (n,4)=(x,y,z,yaw)에서 (v, w)를 duration 동안 유지하면 갈 길 → (n×points, 3)."""
        frac = torch.linspace(1.0 / points, 1.0, points, device=start.device)
        t = duration[:, None] * frac[None, :]                         # (n, points)
        x0, y0, z0, h0 = (start[:, i:i + 1] for i in range(4))
        vv, ww = v[:, None], w[:, None]
        h = h0 + ww * t
        straight = ww.abs() < 1e-3
        safe_w = torch.where(straight, torch.ones_like(ww), ww)
        dx = torch.where(straight, vv * t * torch.cos(h0), vv / safe_w * (torch.sin(h) - torch.sin(h0)))
        dy = torch.where(straight, vv * t * torch.sin(h0), vv / safe_w * (torch.cos(h0) - torch.cos(h)))
        pts = torch.stack([x0 + dx, y0 + dy, (z0 + CommandMarkers.Z_PATH).expand_as(dx)], dim=-1)
        return pts.reshape(-1, 3)


    def draw(self, env, path_w=None) -> None:
        """env(BaseEnv)의 지금 명령 · 실제 속도 · path_w를 그린다 — task env의 debug_vis_draw()에서."""
        d = env.robot.data
        v, w = env.true_twist()
        self.update(d.root_pos_w.torch, self.yaw_of(d.root_quat_w.torch), env.cmd[:, 0], env.cmd[:, 1], v, w, path_w)
