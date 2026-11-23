"""이 프로젝트의 로봇 (실기 TurtleBot3 Waffle Pi 개조형)."""
from .turtlebot3 import TurtleBot3
from .robot import (
    DESCRIPTION_DIR,
    LIDAR_MOUNT_HEIGHT,
    LIDAR_MOUNT_X,
    MAX_WHEEL,
    OPENCR_PORT,
    URDF_PATH,
    WHEEL_RADIUS,
    WHEEL_SEPARATION,
    RealTurtleBot3,
)

__all__ = [
    "DESCRIPTION_DIR",
    "LIDAR_MOUNT_HEIGHT",
    "LIDAR_MOUNT_X",
    "MAX_WHEEL",
    "OPENCR_PORT",
    "URDF_PATH",
    "WHEEL_RADIUS",
    "WHEEL_SEPARATION",
    "RealTurtleBot3",
    "TurtleBot3",
]
