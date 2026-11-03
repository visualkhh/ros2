"""beebot 숫자 — 표준 라이브러리만. ROS 쪽과 같아야 한다 (바꾸면 양쪽 같이):
ros2_ws/src/tb3_bringup/config/robot.yaml (바퀴) · opencr_driver.wheel_speeds (8 rad/s 비례 축소) · rl_correction corrector (max_lin/max_ang).
모양은 urdf.xacro (ROS용 tb3_bringup/urdf/robot.urdf.xacro와 의도적으로 나눈 파일, 내용은 같음 — robots/check_same.sh).
"""
WHEEL_RADIUS = 0.033   # [m]
WHEEL_SEP = 0.287      # [m] (드라이버 기준값. URDF 바퀴 y ±0.144)
MAX_LIN = 0.26         # [m/s]
MAX_ANG = 1.82         # [rad/s]
MAX_WHEEL = 8.0        # [rad/s] 펌웨어 VEL_LIMIT_RAD_S · 드라이버 max_wheel_speed
DIAG_RADIUS = 0.5      # [m] 텔레오퍼 사선 원호 (tool/cli/op/bus.py DIAG_RADIUS)
