"""rl_maze_beebot — beebot으로 미로 주행 (rl_maze의 미로·보상·동적 장애물 그대로, 로봇만 beebot).

  - 로봇: beebot USD(Lv2_bee_Assignment) + rl_nav_correction에서 실측한 RESPONSE_*
  - 센서: nvblox `combined_esdf_pointcloud`(static+dynamic 2D ESDF height slice, 0.03~0.30 m)를 360방향으로.
          beebot `rl_nav_controller.cpp`의 points 경로와 같은 360방향 가상 스캔.
  - 관측 375 = [웨이포인트 10, 스캔 360, v/w/gyro 3, 이전 행동 2] — beebot plugin과 같은 계약.
  - nvblox를 dynamic mapping으로 올려 사람/공도 combined ESDF slice에 포함한다.
  - 배포: Lv2_bee_Assignment `ml`의 `rl_nav_controller` (scan_source=points,
          points_topic=/nvblox_node/combined_esdf_pointcloud, model_path=rl_nav_controller/policy.onnx)
"""
from __future__ import annotations

import os

from isaaclab.utils import configclass
from training.rl_maze.env_cfg import RlMazeEnvCfg


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
    w_collide = 20.0               # 벽/사람/공 실제 충돌 (끝)
    w_smooth = 0.05                # × |행동 변화|² — 노이즈가 커지는 것에 비용
    # 인플레이션 점군과 로봇 몸이 겹칠 때의 작은 패널티 (실제 충돌은 위 w_collide). 매 스텝이라 진행 보상(최대 0.13)보다 작게.
    # 충돌 반경(collide_r)은 로봇 반경이라 보상용 임의 여유가 아니다.
    w_inflate = 0.05
    w_dynamic_inflate = 0.05
    # 사람·공 회피 (reward.py avoid): near 안쪽에서 가까워지면 −, 멀어지면 + (potential 차이). 공 속도 1.2 m/s × 반응 0.5 s ≈ 0.6 m + 여유
    w_avoid = 5.0                  # 접촉(collide_r)까지 다 다가가면 총 −5 (충돌 −20과 별개) · 도망가면 같은 만큼 +
    avoid_near = 1.0               # [m] 이 안에서만 (로봇 중심 → 사람/공 표면)
    # 정책 원출력이 [-1, 1]을 넘은 크기 — env가 clamp해도 mean이 무한히 커지지 않게
    w_action = 0.02
    # 실사형 동적 장애물 — 충돌하면 에피소드 종료(재스폰 없음), 로봇 앞 텔레포트도 없음.
    dynamic_respawn_s = 1e9                               # 8초마다 사람이 갑자기 나타나는 연출 끔
    dynamic_block_teleport = False                        # 로봇을 막는 사람/공을 강제 텔레포트하지 않음
    # (벽에 끼어 못 움직이는 사람만 2초 뒤 다른 곳으로 이동 — 미로 밖으로 나가는 것 방지)
