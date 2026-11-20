"""제어 보드 종류: controller.py(인터페이스), 모델별 구현은 모델 이름 파일 (opencr.py ...). diff_drive.py는 보드가 쓰는 차동 구동 계산(구동부)이다."""
from .controller import Controller

__all__ = ["Controller"]
