"""rl_maze_beebot — beebot으로 미로 주행 (rl_maze의 미로·보상·동적 장애물 그대로, 로봇만 beebot).

  - 로봇: beebot USD(Lv2_bee_Assignment) + rl_nav_correction에서 실측한 RESPONSE_*
  - 센서: beebot `rl_nav_controller.cpp`의 point cloud 경로와 같은 36방향 가상 스캔.
          pole_front_cam은 55° 전방 카메라라, 그 밖의 방향은 lidar_max(3.5 m)로 둔다.
  - 관측 51 = [웨이포인트 10, 스캔 36, v/w/gyro 3, 이전 행동 2] — beebot plugin과 같은 계약.
  - 배포: Lv2_bee_Assignment `ml`의 `rl_nav_controller` (scan_source=points, model_path=rl_nav_controller/policy.onnx)
"""
from __future__ import annotations

import os

from isaaclab.utils import configclass
from training.rl_maze.env_cfg import RlMazeEnvCfg


@configclass
class RlMazeBeebotEnvCfg(RlMazeEnvCfg):
    robot = os.path.dirname(os.path.abspath(__file__))   # 이 폴더의 robot_cfg.py + usda/robot.usda
    maze_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "usda", "maze")
    observation_space = 2 * 5 + 36 + 5                    # 51 — inflation 입력 없음 (beebot plugin과 동일)
    camera_fov_deg = 55.0                                 # pole_front_cam 수평 FOV — 이 밖은 lidar_max
    collide_r = 0.18                                      # beebot Nav2 robot_radius
