"""CalibrationEnv — 이 task의 calibration sim 측정 환경 (= DefaultEnv: 빈 평면 · 순수 sim, 보정 없음). 학습 전 1단계.

측정 장면을 바꾸려면 여기서 (DefaultEnv · BaseEnv 메서드를 덮어쓴다). Isaac 앱이 켜진 뒤 import.
"""
from __future__ import annotations

import math

import torch

from envs.default.env import DefaultEnv
from training.rl_correction_waypoint.markers import CommandMarkers


class CalibrationEnv(DefaultEnv):
    path_w = None   # 노란 점 = 목표 경로 (m, 3) — set_path()

    def start(self, path=None) -> float:
        """측정 시작 자세: reset → 0.5 s 정지 (스폰 높이에서 떨어져 바퀴 · 캐스터로 서게) → 노란 점 = 목표 경로. 높이 [m] 반환."""
        self.reset()
        zero = torch.zeros(self.num_envs, 2, device=self.device)
        for _ in range(10):
            self.step(zero)
        if path and self.cfg.debug_vis:
            self.set_path(path)
        return float(self.robot.data.root_pos_w.torch[0, 2])

    def set_path(self, xy, i: int = 0) -> None:
        """목표 경로(명령의 path(): 시작 자세 기준 [(x 앞, y 왼쪽)])를 env i 로봇의 지금 자세로 옮겨 노란 점으로. None이면 끔."""
        if not xy:
            self.path_w = None
            return
        d = self.robot.data
        p0, q = d.root_pos_w.torch[i].tolist(), d.root_quat_w.torch[i].tolist()   # xyzw
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        pts = []
        for (x0, y0), (x1, y1) in zip(xy, xy[1:]):   # 3 cm 간격 점
            k = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.03))
            pts += [(x0 + (x1 - x0) * j / k, y0 + (y1 - y0) * j / k) for j in range(k)]
        c, s = math.cos(yaw), math.sin(yaw)
        self.path_w = torch.tensor([[p0[0] + c * x - s * y, p0[1] + s * x + c * y, p0[2]] for x, y in pts], device=self.device)

    # ------------------------------------------------------------------ 화면 (cfg.debug_vis일 때 BaseEnv가 부른다)
    def debug_vis_setup(self, visible: bool) -> None:
        if visible and getattr(self, "markers", None) is None:
            self.markers = CommandMarkers()
        if getattr(self, "markers", None) is not None:
            self.markers.set_visible(visible)

    def debug_vis_draw(self) -> None:
        self.markers.draw(self, self.path_w)
