"""액추에이터 값. 한 파일 한 클래스.

calibration sim(DefaultScene)과 학습 env(rl_correct_env_cfg)가 **둘 다 이 값**을 쓴다 — 같은 로봇이어야
calibration에서 잰 sim fit이 곧 학습 env의 raw 응답이 된다.
damping은 실측으로 정한다: ./run-calibration.sh sensor-sync-sim --viz none --damping <값> 으로 몇 개 재서
real fit과 비교 (calibration/EXPERIMENT.md "damping 정하기"). 정해지면 여기 기본값만 바꾼다.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ActuatorParam:
    damping: float = 100.0       # 바퀴 속도 서보 이득 (ImplicitActuator, stiffness 0) — 실측 전 임시값
    effort: float = 3.0          # [N·m] XM430-W210 stall 근사
    velocity_limit: float = 9.0  # [rad/s] MAX_WHEEL(8) + 여유 — 포화는 바퀴 모델이 드라이버처럼 비례 축소로 직접
