"""rl_maze_beebot — beebot으로 미로 주행 (rl_maze의 미로·보상·동적 장애물 그대로, 로봇만 beebot).

  - 로봇: beebot USD(Lv2_bee_Assignment) + rl_nav_correction에서 실측한 RESPONSE_*
  - 센서: nvblox `static_esdf_pointcloud`(2D ESDF height slice, 0.03~0.6 m)를 360방향으로.
          beebot `rl_nav_controller.cpp`의 points 경로와 같은 36→360방향 가상 스캔.
  - 관측 51→375 = [웨이포인트 10, 스캔 360, v/w/gyro 3, 이전 행동 2] — beebot plugin과 같은 계약.
  - nvblox가 static_tsdf라 사람/공은 SLAM map에 없음 → 학습도 static 미로만 (n_people=0, n_balls=0).
    사람/공까지 policy가 보려면 nvblox mapping_type=dynamic + combined_esdf_pointcloud.
  - 배포: Lv2_bee_Assignment `ml`의 `rl_nav_controller` (scan_source=points,
          points_topic=/nvblox_node/static_esdf_pointcloud, model_path=rl_nav_controller/policy.onnx)
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
    n_people = 0                                          # nvblox static_tsdf에는 사람이 없음
    n_balls = 0
