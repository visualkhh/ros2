"""rviz 같은 화면: Viz(kros)가 토픽(tf, robot_description, scan, imu, odom)만 구독해서 그린다. 월드/하드웨어는 모른다 (viewer/는 진실을 보는 창, viz/는 로봇이 내보낸 값을 보는 창)."""
from .viz import Viz

__all__ = ["Viz"]
