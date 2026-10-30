"""goal 환경 config — 목표 변위까지 이동하는 바퀴 보정 정책.

기존 `RlCorrectEnvCfg`(속도 추종)를 상속하고, 목표점·보상·관측만 goal용으로 바꾼다.
"""
from __future__ import annotations

from isaaclab.utils import configclass

from training.rl_correction_wheel_drift.task.rl_correct_env_cfg import RlCorrectEnvCfg


@configclass
class GoalEnvCfg(RlCorrectEnvCfg):
    episode_length_s = 12.0
    observation_space = 10  # [gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_dx, prev_dw]

    # 에피소드 시작 시 목표 변위 (시작 자세 기준, m)
    goal_radius_range = (0.4, 1.2)
    goal_tol = 0.05           # 성공 반경 [m]
    k_dist = 1.0              # 목표까지 거리 → 전진 속도
    k_heading = 2.0           # 목표 방향 오차 → 회전 속도
    turn_thresh = 0.5         # 이보다 크면 제자리 회전 [rad]

    # 보상
    goal_progress_weight = 1.0
    lateral_weight = 0.5      # 목표 직선에서 옆으로 새는 벌점
    lateral_deadband_m = 0.05  # 이 정도 옆 오차는 벌점 없음
    # action_penalty · smooth_penalty는 부모 값을 그대로 사용
