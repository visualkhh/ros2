"""rl_maze 설정 — 미로 안에서 웨이포인트(경로)를 따라 가장 빨리 도착하는 주행 정책 (nav2 controller 자리). 로봇과 상관없음."""
from __future__ import annotations

import math
import os

from isaaclab.envs import ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from envs.base_env_cfg import BaseEnvCfg
from training.rl_maze.ground import GroundCfg


@configclass
class RlMazeEnvCfg(BaseEnvCfg):
    """미로 하나에 로봇 여러 대 (env끼리는 충돌 안 함 — 서로 안 보이는 겹친 세상). 관측 52 / 행동 2.

    관측 = [다음 웨이포인트 K개 (로봇 기준 x, y) · 라이다 N개 (0~1) · v_enc · w_enc · w_gyro · 이전 행동 2 · inflation]
    행동 = [v, w] 정규화 [-1, 1] → × (max_lin, max_ang) — 정책이 직접 운전 (보정 아님).
    """
    robot = os.path.dirname(os.path.abspath(__file__))   # 이 폴더의 robot_cfg.py + usda/robot.usda
    maze_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "usda", "maze")  # 미로 USD · grid.npz · paths.npz
    episode_length_s = 50.0
    random_yaw = False
    action_space = 2
    observation_space = 2 * 5 + 36 + 3 + 2 + 1   # +1 = lidar inflation (사용자 입력)
    state_space = 0

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=256, env_spacing=0.0, replicate_physics=True)   # 모두 같은 미로
    viewer: ViewerCfg = ViewerCfg(eye=(0.0, -5.0, 6.0), lookat=(0.0, 0.0, 0.0))   # 미로 전체
    snap_origin, snap_eye, snap_lookat = "world", (0.0, -5.0, 6.0), (0.0, 0.0, 0.0)   # 스크린샷도 미로 전체

    # 웨이포인트 (command.py · maze_grid.py)
    lookahead = 5                  # 관측에 넣는 다음 웨이포인트 개수 (K)
    wp_tol = 0.15                  # [m] 이 안에 들어오면 다음 웨이포인트
    wp_scale = 2.0                 # [m] 웨이포인트 관측 나눔 (대략 0~1)

    # 라이다 (실기기 LDS-03 /scan 자리 — 미로 격자 + 동적 장애물에 광선)
    lidar_rays = 36                # 360°를 고르게
    lidar_max = 3.5                # [m]
    lidar_noise = 0.01             # [m] 거리 잡음
    collide_r = 0.17               # [m] 로봇 중심이 벽/사람 표면에서 이보다 가까우면 충돌 (끝)
    lidar_vis = False              # 디버그 화면에 라이다 점군 (L 키와 같은 것 — 학습 --viz에서 env.lidar_vis=true)
    lidar_vis_rays = 360           # L 점군 시각화 방향 수 (정책 입력은 36 그대로; dense raycast)

    # LiDAR 인플레이션 — 사용자가 주는 값(런타임 ROS 파라미터와 같은 단위 m).
    #   obs 마지막 1개 = inflation / inflation_scale · 스캔 = clamp(거리 - inflation, 0, lidar_max)
    #   학습은 env마다 에피소드 시작 때 범위에서 뽑는다 (한 에피소드 안에서는 고정).
    inflation_range = (0.20, 0.24)  # [m] 학습 범위 = 사용자 지원 범위 — 경로 최소 = max + path_margin(0.34)
    inflation_scale = 0.50          # 관측 정규화 상수 (사용자 값 아님) — 0.24 → 0.48
    path_margin = 0.10              # [m] 경로 최소 = max(inflation_range) + path_margin — 가상 간격 확보

    # 동적 장애물 — 미로 하나(env_spacing=0)라 전역 1세트. 사람은 걷고, 공은 튄다.
    n_people = 3                   # 걷는 사람 수 (전역)
    n_balls = 2                    # 공 수 (전역)
    person_radius = 0.26           # [m] 충돌 · 라이다 원 반경 (몸 + 팔)
    person_speed = (0.25, 0.65)    # [m/s] 걷는 속도 범위 (로봇 0.26 m/s보다 조금 빠른 정도)
    person_pause_chance = 0.35     # 방향을 바꿀 때 이 확률로 잠깐 멈춘다
    person_pause_s = (1.0, 3.0)    # [s] 멈추는 시간
    ball_radius = 0.13             # [m]
    ball_speed = (0.40, 1.20)      # [m/s]
    dynamic_respawn_s = 8.0        # [s] 사람 한 명이 갑자기 다시 나타나는 간격 (로봇 앞쪽 우선)
    dynamic_block_teleport = True  # 로봇 앞을 오래 막는 사람/공을 텔레포트 (play/소수 env) — 실사형 task는 False
    dynamic_spawn_min = 1.8        # [m] 갑자기 나타날 때 로봇 앞 최소 거리
    dynamic_spawn_max = 3.2        # [m] 최대 거리 (라이다 3.5 m 안)
    spawn_grace_steps = 12         # [스텝] 리셋 직후 충돌 판정 유예 (0.6 s — 사람이 겹쳐 있어도 출발 가능)

    # 배경 — 마찰 패치 (ground.py). 미로 전체에 전역 1세트.
    ground: GroundCfg = GroundCfg()

    # 행동
    v_min_ratio = -0.2             # 뒤로는 max_lin의 20%까지
    pose_noise = 0.02              # [m] 위치 관측 잡음 (실기기 = 위치 추정)
    yaw_noise = math.radians(2.0)

    # 보상 — 안전 최우선 + 가장 빨리 도착:
    #   진행 + 도착 보너스 − 시간 − 충돌(끝) − 인플레이션 안쪽 겹침 − 떨림 − 후진
    #   인플레이션 점군은 soft cost: 실제 라이다/충돌(0.17 m)은 hard, 겹치면 감점만.
    w_progress = 2.0               # × [m] 줄인 거리 (최대한 빨리 = 진행을 크게)
    w_reach = 2.0                  # 웨이포인트 하나 통과
    w_goal = 100.0                 # 마지막 도착
    w_time = 0.10                  # 매 스텝 (20 Hz → 2/s)
    w_collide = 200.0              # 벽/사람/공 실제 충돌 (끝)
    w_wall = 0.2                   # 매 스텝 × 인플레이션 안쪽 겹침 [0,1] (soft)
    w_dynamic = 0.3                # 매 스텝 × 사람/공 인플레이션 겹침 [0,1] (soft)
    w_smooth = 0.02                # × |행동 변화|²
    w_back = 0.5                   # × 뒤로 간 속도 [m/s]
