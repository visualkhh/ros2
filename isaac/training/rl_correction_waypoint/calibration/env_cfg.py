"""CalibrationEnvCfg — 이 task의 calibration sim 측정 설정 (= DefaultEnvCfg: 빈 평평한 바닥). 로봇은 실행기가 이 task 폴더로 채운다."""
from __future__ import annotations

from isaaclab.utils import configclass

from envs.default.env_cfg import DefaultEnvCfg
from training.rl_correction_waypoint.ground import PLANE, GroundCfg


@configclass
class CalibrationEnvCfg(DefaultEnvCfg):
    ground: GroundCfg = PLANE   # 측정은 평면 (순수 sim) — run-world --command는 학습 배경 GroundCfg()로 바꿔 씀
