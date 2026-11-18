"""rl_maze_beebot — beebot으로 미로 주행 (rl_maze의 미로 그대로, 로봇만 beebot · 사람/공 같은 동적 장애물 없음).

  - 로봇: beebot USD(Lv2_bee_Assignment) + rl_nav_correction에서 실측한 RESPONSE_*
  - 센서: nvblox `combined_esdf_pointcloud`(2D ESDF height slice, 0.03~0.30 m)를 360방향으로.
          beebot `rl_nav_controller.cpp`의 points 경로와 같은 360방향 가상 스캔.
  - 관측 375 = [웨이포인트 10, 스캔 360, v/w/gyro 3, 이전 행동 2] — beebot plugin과 같은 계약.
  - 배포: Lv2_bee_Assignment `ml`의 `rl_nav_controller` (scan_source=points,
          points_topic=/nvblox_node/combined_esdf_pointcloud, model_path=rl_nav_controller/policy.onnx)
"""
from __future__ import annotations

import os

from isaaclab.utils import configclass
from training.rl_maze.env_cfg import RlMazeEnvCfg
from training.rl_maze.ground import GroundCfg


@configclass
class RlMazeBeebotEnvCfg(RlMazeEnvCfg):
    robot = os.path.dirname(os.path.abspath(__file__))   # 이 폴더의 robot_cfg.py + usda/robot.usda
    maze_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "usda", "maze")
    observation_space = 2 * 5 + 360 + 5                   # 375 — beebot plugin (nvblox 360 rays)과 동일
    lidar_rays = 360                                      # nvblox 0.05 m voxel / 3.5 m → 1°(360)가 최대 의미 해상도
    lidar_vis_rays = 360                                  # L 시각화도 같은 360방향
    collide_r = 0.18                                      # beebot Nav2 robot_radius
    # 인플레이션은 실기 파라미터(lidar_inflation)와 같은 단위. 사용자가 1~5 cm로 바꿀 수 있으므로
    # 학습도 같은 범위(0.01~0.05 m)를 랜덤으로 커버한다 — 좁은 범위면 실기 설정 변경에 일반화가 안 된다.
    inflation_range = (0.01, 0.05)
    path_margin = 0.32                                    # max(inflation) + 0.32 = 경로 최소 0.37 m 유지
    # ── 보상 (rl_maze 값을 beebot 학습용으로 다시 맞춤) ──────────────────────────────────────────────
    # 2026-11-09 overnight2 (230 iter)에서: 가치 손실 300~600 · std 0.6→1.9로 계속 증가 · goal 0 · progress 보상 0.
    #   충돌의 96 %가 사람·공 (전역 3명+2개가 512대와 같은 미로) — 초반 정책은 피할 수 없어 −200이 행동과 무관한 노이즈로 들어갔다.
    #   per 스텝 한도로 보면 진행 보상(2.0 × 0.013 m = 0.026)이 시간 패널티(0.1)보다 작아 움직여도 이득이 안 보였다.
    # 그래서: 전체 크기를 낮추고(충돌 −200→−20), 매 스텝 나오는 진행 보상을 키우고(×5), 시간 패널티를 줄이고(0.1→0.02),
    #   떨림 패널티를 올려(0.02→0.05) 노이즈(std)가 커지는 쪽에 비용을 준다.
    #   전진 1 m ≈ progress 10 + reach 3.3(0.3 m 간격) = 13.3, 0.2 m/s로 가면 시간 −2/m → 움직이면 이득. 충돌 −20 ≈ 1.5 m어치 + 그 뒤 보상을 잃음.
    w_progress = 10.0              # × [m] 줄인 거리 — 매 스텝 나오는 주 신호
    w_reach = 1.0                  # 웨이포인트 하나 통과 (0.3 m 간격이라 1 m ≈ 3.3)
    w_goal = 20.0                  # 마지막 도착
    w_time = 0.02                  # 매 스텝 (20 Hz → 0.4/s)
    w_collide = 20.0               # 벽 실제 충돌 (끝)
    w_smooth = 0.05                # × |행동 변화|² — 노이즈가 커지는 것에 비용
    # 인플레이션 점군과 로봇 몸이 겹칠 때의 작은 패널티 (실제 충돌은 위 w_collide). 매 스텝이라 진행 보상(최대 0.13)보다 작게.
    # 충돌 반경(collide_r)은 로봇 반경이라 보상용 임의 여유가 아니다.
    w_inflate = 0.05
    # 주행 특화 (reward.py speed · heading · spin) — 진행 보상(매 스텝 ≈0.13)을 보조하는 작은 값
    w_speed = 0.05                 # × (전진속도/max_lin) × cos(길 방향 오차) — 길 쪽을 보고 빨리 갈수록 +
    w_heading = 0.02               # × (1−cos 오차)/2 — 몸이 길에서 틀어질수록 −
    w_spin = 0.02                  # × (각속도/max_ang)² — 제자리 · 지그재그 회전 −
    w_center = 0.03                # × |l−r|/(l+r) — 복도(좌우 벽이 다 center_range 안)에서 한쪽 벽에 치우칠수록 −
    center_range = 1.0             # [m] 좌우 벽이 둘 다 이 안일 때만 중앙 보상을 적용
    w_path = 0.5                   # × 경로 선분까지 거리 [m] (허용 오차 없음, 최대 0.5 m) — 10 cm 벗어나면 −0.05/스텝. 2.0은 진행 보상(0.13)보다 커서 정책이 서 있게 굳었음 (path1k 112 iter: 에피소드 1000 · waypoints 0.1 · goal 0)
    # 정책 원출력이 [-1, 1]을 넘은 크기 — env가 clamp해도 mean이 무한히 커지지 않게
    w_action = 0.02
    # 동적 장애물(사람·공) 없음 — rl_maze 기본값(3명·2개)을 끈다. env는 0개여도 그대로 동작.
    n_people = 0
    n_balls = 0
    # 마찰 패치 — rl_maze 기본(끈적 1.40/1.20 · 미끄럼 0.10/0.06)보다 약하게: 바퀴(1.0)와 average라 실제 적용은 끈적 1.10/1.03 · 미끄럼 0.65/0.60
    ground: GroundCfg = GroundCfg(sticky_friction=(1.20, 1.05), slippery_friction=(0.30, 0.20))
