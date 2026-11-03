"""rl_correction_waypoint — 랜덤 웨이포인트를 이탈 없이 따라가기 (기본 제어기 + 정책 보정). Isaac Lab task 등록 — 로봇마다 이름 하나.

  RlCorrectionWaypoint-Beebot   env = RlCorrectionWaypointEnv · env_cfg = RlCorrectionWaypointEnvCfg (robot="beebot" + beebot 실측 중심)
  새 로봇: env_cfg.py에 XWaypointEnvCfg(RlCorrectionWaypointEnvCfg)(robot · 실측만) + 아래에 gym.register(id="RlCorrectionWaypoint-X", …).
  실험용으로 로봇만 바꾸기: ./run-training.sh env.robot=<이름 또는 폴더>
  학습: ./run-training.sh  (= Isaac Lab scripts/reinforcement_learning/train.py --task RlCorrectionWaypoint-Beebot --rl_library rsl_rl)
  재생: ./run-training.sh play --checkpoint <model_N.pt>  (= play.py — exported/policy.onnx 내보내기)
등록은 문자열 위치만 적는다 — 이 파일은 Isaac 앱 전에 import된다 (공식 스크립트 --external_callback).
"""
import gymnasium as gym

gym.register(
    id="RlCorrectionWaypoint-Beebot",
    entry_point=f"{__name__}.env:RlCorrectionWaypointEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:RlCorrectionWaypointEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agent:RlCorrectionWaypointAgentCfg",
    },
)
