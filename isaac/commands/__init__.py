"""명령 경로 (Isaac Lab의 command 개념): 시각 t → 명령 (v, w). env는 명령을 받아 바퀴를 돌릴 뿐이다.

BaseCommand를 상속해 만든다. base · square는 표준 라이브러리만 (실기기 측정이 Isaac 없이 ROS에서 돈다).
keyboard는 Isaac 창이 필요 (AppLauncher 뒤에 import).
"""
import importlib

CMDS = {"keyboard": "commands.keyboard:KeyboardCommand", "square": "commands.square:SquareCommand"}


def command_class(name: str):
    """'square' 같은 이름(CMDS) 또는 'pkg.mod:Class' → BaseCommand 클래스 (import는 이때 — keyboard는 Isaac 뒤에)."""
    spec = CMDS.get(name, name)
    mod, _, cls = spec.partition(":") if ":" in spec else spec.rpartition(".")
    return getattr(importlib.import_module(mod), cls)
