"""이 프로젝트(kos_ws)가 소유하는 하드웨어 사본. kros 공용 부품과 같은 인터페이스를 쓴다.

- opencr/: OpenCR 시리얼 (firmware/opencr/tb3_wheels)
- lds_03.py: LDS-03 (COIN-D4) 라이다
"""
from .lds_03 import Lds03, SerialLds03
from .opencr import OpenCrSerial

__all__ = ["Lds03", "OpenCrSerial", "SerialLds03"]
