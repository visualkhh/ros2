"""CalibrationCommand — 이 task의 calibration 측정 명령 (= 네모, commands/square.SquareCommand). 학습 전 1단계.

측정 경로를 바꾸려면 여기서 (fit · 구간 표는 segs label move_* · turn_*을 쓴다). 표준 라이브러리만 — sim · real · dummy 공용.
"""
from __future__ import annotations

from commands.square import SquareCommand


class CalibrationCommand(SquareCommand):
    """2 s 정지 → 왼쪽 사각형(반시계, 변마다 속도 다르게) → 180° → 같은 사각형 역주행 → 2 s 정지 (기본 54 s)."""
