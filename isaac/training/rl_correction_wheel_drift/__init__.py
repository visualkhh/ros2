"""rl_correction_wheel_drift — 바퀴 드리프트 보정 정책 (beebot). Isaac Lab task 등록 (gym.register).

  학습: ./run-training.sh  (= Isaac Lab scripts/reinforcement_learning/train.py --task Beebot-WheelDrift --rl_library rsl_rl)
  재생: ./run-training.sh play  (= play.py — logs/…/exported/policy.onnx 내보내기)
등록은 문자열 위치만 적는다 — 이 파일은 Isaac 앱 전에 import된다 (공식 스크립트 --external_callback).
"""
import gymnasium as gym

gym.register(
    id="Beebot-WheelDrift",
    entry_point=f"{__name__}.env:RlCorrectEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:RlCorrectEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agent:RlCorrectAgentCfg",
    },
)
