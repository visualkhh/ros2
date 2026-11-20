"""진짜 OpenCR 보드를 시리얼로 쓰는 구현 (OpenCr 시뮬레이션과 같은 자리). 프로토콜은 firmware/opencr/README.md."""
from .opencr_serial import OpenCrSerial

__all__ = ["OpenCrSerial"]
