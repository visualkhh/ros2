"""모터 종류: motor.py(인터페이스), 모델별 구현은 모델 번호 파일 (xl430_w250.py ...). 새 모터는 Motor를 상속한 파일로 여기에 추가한다."""
from .motor import Motor

__all__ = ["Motor"]
