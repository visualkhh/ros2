"""액추에이터 고정값. 한 파일 한 클래스.

calibration sim(DefaultScene)과 학습 env(rl_correct_env_cfg)가 **둘 다 이 값**을 쓴다.
damping/effort는 더 이상 calibration 변수가 아니다 — 실측해도 sim 응답이 변하지 않아서
(--damping/--effort sweep이 no-op) 측정·학습 모두에서 뺐다. 지금은 고정 모델값이다.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ActuatorParam:
    damping: float = 50.0        # 바퀴 속도 서보 이득 (ImplicitActuator, stiffness 0) — 고정
    effort: float = 3.0          # [N·m] XM430-W210 stall 근사 — 고정
    velocity_limit: float = 9.0  # [rad/s] MAX_WHEEL(8) + 여유 — 포화는 바퀴 모델이 드라이버처럼 비례 축소로 직접
