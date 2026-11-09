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
    # 인플레이션 점군과 로봇 몸이 겹칠 때의 작은 패널티 (실제 충돌 −200은 별도).
    # 충돌 반경(collide_r)은 로봇 반경이라 보상용 임의 여유가 아니다.
    w_inflate = 0.5
    w_dynamic_inflate = 0.5
    # 정책 원출력이 [-1, 1]을 넘은 크기 — env가 clamp해도 mean이 무한히 커지지 않게
    w_action = 0.1
    # 중간 웨이포인트 보상 — 기본 2.0보다 크게 (경로를 끝까지 따라가게)
    w_reach = 5.0
    # 실사형 동적 장애물 — 충돌하면 에피소드 종료(재스폰 없음), 로봇 앞 텔레포트도 없음.
    dynamic_respawn_s = 1e9                               # 8초마다 사람이 갑자기 나타나는 연출 끔
    dynamic_block_teleport = False                        # 로봇을 막는 사람/공을 강제 텔레포트하지 않음
    # (벽에 끼어 못 움직이는 사람만 2초 뒤 다른 곳으로 이동 — 미로 밖으로 나가는 것 방지)
