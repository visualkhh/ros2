"""라이다 종류: lidar.py(인터페이스), 모델별 구현은 모델 번호 파일 (lds_01.py ...). 새 라이다는 Lidar를 상속한 파일로 여기에 추가한다."""
from .lidar import Lidar

__all__ = ["Lidar"]
