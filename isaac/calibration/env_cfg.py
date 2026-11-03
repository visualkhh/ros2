"""CalibrationEnvCfg — calibration sim 측정 설정 (= DefaultEnvCfg: 빈 평평한 바닥 · 로봇 robots.DEFAULT). training/<task>/env_cfg.py와 같은 자리."""
from __future__ import annotations

from isaaclab.utils import configclass

from envs.default.env_cfg import DefaultEnvCfg


@configclass
class CalibrationEnvCfg(DefaultEnvCfg):
    pass
