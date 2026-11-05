"""rl_nav_correction — 명령 (v, w)를 몸체가 그대로 따라가게 하는 정책 보정 (nav2 컨트롤러 뒤 보정 노드용 · 관측 6). Isaac Lab task 등록.

  RlNavCorrection   env = RlNavCorrectionEnv · env_cfg = RlNavCorrectionEnvCfg · agent = RlNavCorrectionAgentCfg
  로봇: 이 폴더의 robot_cfg.py + usda/robot.usda (robots.DEFAULT) · 다른 로봇: ./run-training.sh rl_nav_correction env.robot=<이름 또는 폴더>
  (그 로봇의 robot_cfg.py가 바퀴 숫자 · 실측 응답을 채운다 — 실측 응답이 랜덤 범위의 중심)
  학습: ./run-training.sh rl_nav_correction  (= Isaac Lab scripts/reinforcement_learning/train.py --task RlNavCorrection --rl_library rsl_rl)
  재생: ./run-training.sh rl_nav_correction --play --checkpoint <model_N.pt>  (= play.py — exported/policy.onnx 내보내기)
등록은 문자열 위치만 적는다 — 이 파일은 Isaac 앱 전에 import된다 (공식 스크립트 --external_callback).
"""
import gymnasium as gym

gym.register(
    id="RlNavCorrection",
    entry_point=f"{__name__}.env:RlNavCorrectionEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:RlNavCorrectionEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agent:RlNavCorrectionAgentCfg",
    },
)
