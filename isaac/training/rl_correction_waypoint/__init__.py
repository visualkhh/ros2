"""rl_correction_waypoint — 랜덤 웨이포인트를 이탈 없이 따라가기 (기본 제어기 + 정책 보정). Isaac Lab task 등록.

  RlCorrectionWaypoint   env = RlCorrectionWaypointEnv · env_cfg = RlCorrectionWaypointEnvCfg · agent = RlCorrectionWaypointAgentCfg
  로봇: 이 폴더의 robot_cfg.py + usda/robot.usda (robots.DEFAULT) · 다른 로봇: ./run-training.sh env.robot=<이름 또는 폴더>
  (그 로봇의 robot_cfg.py가 바퀴 숫자 · 실측 응답을 채운다 — 실측 응답이 랜덤 범위의 중심)
  학습: ./run-training.sh  (= Isaac Lab scripts/reinforcement_learning/train.py --task RlCorrectionWaypoint --rl_library rsl_rl)
  재생: ./run-training.sh play --checkpoint <model_N.pt>  (= play.py — exported/policy.onnx 내보내기)
등록은 문자열 위치만 적는다 — 이 파일은 Isaac 앱 전에 import된다 (공식 스크립트 --external_callback).
"""
import gymnasium as gym

gym.register(
    id="RlCorrectionWaypoint",
    entry_point=f"{__name__}.env:RlCorrectionWaypointEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:RlCorrectionWaypointEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agent:RlCorrectionWaypointAgentCfg",
    },
)
