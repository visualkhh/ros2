"""python -m calibration <command> [옵션...] — 명령마다 스크립트 모듈의 main(argv). 런타임별로 lazy import
(sim은 IsaacLab venv, real은 ROS 환경, dummy는 표준 라이브러리만)."""
import importlib
import sys

COMMANDS = {"sensor-sync-sim": "sim_interface", "sensor-sync-real": "real_interface", "dummy": "dummy_interface"}

args = sys.argv[1:]
if not args or args[0] not in COMMANDS:
    print("사용법: python -m calibration <" + " | ".join(COMMANDS) + "> [옵션...]")
    raise SystemExit(0 if args[:1] in (["-h"], ["--help"]) else 2)
raise SystemExit(importlib.import_module(f"calibration.{COMMANDS[args[0]]}").main(args[1:]) or 0)
