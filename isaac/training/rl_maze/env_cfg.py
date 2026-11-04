"""rl_maze 설정 — 미로 안에서 웨이포인트(경로)를 따라 가장 빨리 도착하는 주행 정책 (nav2 controller 자리). 로봇과 상관없음."""
from __future__ import annotations

import math
import os

from isaaclab.envs import ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from envs.base_env_cfg import BaseEnvCfg


@configclass
class RlMazeEnvCfg(BaseEnvCfg):
    """미로 하나에 로봇 여러 대 (env끼리는 충돌 안 함 — 서로 안 보이는 겹친 세상). 관측 51 / 행동 2.

    관측 = [다음 웨이포인트 K개 (로봇 기준 x, y) · 라이다 N개 (0~1) · v_enc · w_enc · w_gyro · 이전 행동 2]
    행동 = [v, w] 정규화 [-1, 1] → × (max_lin, max_ang) — 정책이 직접 운전 (보정 아님).
    """
    robot = os.path.dirname(os.path.abspath(__file__))   # 이 폴더의 robot_cfg.py + usda/robot.usda
    episode_length_s = 60.0
    random_yaw = False
    action_space = 2
    observation_space = 2 * 5 + 36 + 3 + 2
    state_space = 0

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=256, env_spacing=0.0, replicate_physics=True)   # 모두 같은 미로
    viewer: ViewerCfg = ViewerCfg(eye=(0.0, -5.0, 6.0), lookat=(0.0, 0.0, 0.0))   # 미로 전체
    snap_origin, snap_eye, snap_lookat = "world", (0.0, -5.0, 6.0), (0.0, 0.0, 0.0)   # 스크린샷도 미로 전체

    # 웨이포인트 (command.py · maze_grid.py)
    lookahead = 5                  # 관측에 넣는 다음 웨이포인트 개수 (K)
    wp_tol = 0.15                  # [m] 이 안에 들어오면 다음 웨이포인트
    wp_scale = 2.0                 # [m] 웨이포인트 관측 나눔 (대략 0~1)

    # 라이다 (실기기 LDS-03 /scan 자리 — 미로 격자에 광선)
    lidar_rays = 36                # 360°를 고르게
    lidar_max = 3.5                # [m]
    lidar_noise = 0.01             # [m] 거리 잡음
    collide_r = 0.17               # [m] 로봇 중심이 벽에서 이보다 가까우면 충돌 (끝)

    # 행동
    v_min_ratio = -0.2             # 뒤로는 max_lin의 20%까지
    pose_noise = 0.02              # [m] 위치 관측 잡음 (실기기 = 위치 추정)
    yaw_noise = math.radians(2.0)

    # 보상 — 가장 빨리 · 짧게: 진행(웨이포인트 쪽으로 줄인 거리) + 도착 보너스 − 시간 − 충돌
    w_progress = 10.0              # × [m] 줄인 거리
    w_reach = 1.0                  # 웨이포인트 하나 통과
    w_goal = 10.0                  # 마지막 도착
    w_time = 0.05                  # 매 스텝
    w_collide = 10.0               # 벽 충돌 (끝)
    w_smooth = 0.01                # × |행동 변화|²
    w_back = 0.5                   # × 뒤로 간 속도 [m/s]
