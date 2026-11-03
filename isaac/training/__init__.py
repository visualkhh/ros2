"""학습 task들 (Isaac Lab task 폴더 관례). 각 task 폴더의 __init__.py가 gym.register로 이름을 등록한다.

공식 학습 스크립트가 `--external_callback training.register`로 register()를 불러 우리 task를 등록한다 (run-training.sh).
"""
import importlib
import os

TASKS_DIR = os.path.dirname(os.path.abspath(__file__))


def register() -> None:
    """training/ 아래 task 폴더(__init__.py가 있는 곳)를 전부 import → gym.register가 돈다."""
    for d in sorted(os.listdir(TASKS_DIR)):
        if os.path.isfile(os.path.join(TASKS_DIR, d, "__init__.py")):
            importlib.import_module(f"training.{d}")
