"""rl_correction_wheel_drift — 바퀴 드리프트 보정 정책. Isaac Lab task 등록 (gym.register) — 로봇마다 이름 하나.

  Beebot-WheelDrift   env = RlCorrectionWheelDriftEnv · env_cfg = RlCorrectionWheelDriftEnvCfg (robot="beebot" + beebot 실측 중심)
  새 로봇: env_cfg.py에 XWheelDriftEnvCfg(RlCorrectionWheelDriftEnvCfg)(robot · 실측만) + 아래에 gym.register(id="X-WheelDrift", …).
  실험용으로 로봇만 바꾸기: ./run-training.sh env.robot=<이름 또는 폴더>

  학습: ./run-training.sh  (= Isaac Lab scripts/reinforcement_learning/train.py --task Beebot-WheelDrift --rl_library rsl_rl)
  재생: ./run-training.sh play  (= play.py — logs/…/exported/policy.onnx 내보내기)
등록은 문자열 위치만 적는다 — 이 파일은 Isaac 앱 전에 import된다 (공식 스크립트 --external_callback).
"""
import gymnasium as gym

gym.register(
    id="Beebot-WheelDrift",
    entry_point=f"{__name__}.env:RlCorrectionWheelDriftEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:RlCorrectionWheelDriftEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agent:RlCorrectionWheelDriftAgentCfg",
    },
)
