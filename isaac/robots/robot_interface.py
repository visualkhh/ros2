"""RobotInterface — sim · real · 가짜 로봇을 같은 방식으로: write(v, w)로 명령을 보내고 read()로 지금 상태를 읽는다.

ros2_control hardware_interface의 read() · write()와 같은 역할. 명령 경로(BaseCommand)나 기록 형식은 모른다.
  SimRobot    robots/sim_robot.py (공용) · <로봇 폴더>/sim_robot.py (상속)   env.step([[v, w]])  — Isaac
  DummyRobot  robots/dummy_robot.py                   가짜 모터 계산            — 표준 라이브러리만
  RealRobot   <로봇 폴더>/real_robot.py (robots.load(robot, "real_robot"))   그 로봇의 ROS 토픽
이 파일은 표준 라이브러리만.
"""
from __future__ import annotations

STATE_KEYS = ("lin_out", "ang_out", "odom_x", "odom_y", "odom_yaw", "v_enc", "w_enc", "w_gyro",
              "wheel_l_vel", "wheel_r_vel", "wheel_l_pos", "wheel_r_pos")
"""read()가 돌려주는 값. lin_out · ang_out = 실제로 나간 명령 [m/s · rad/s] · odom = 엔코더 적분 자세 · v_enc · w_enc = 엔코더 속도 ·
w_gyro [rad/s] · wheel_* = 바퀴 각속도 [rad/s] · 각도 [rad]. real은 메시지 시각 t_ros · t_odom · t_imu · t_joint [s]도 준다."""


class RobotInterface:
    def __init__(self, dt: float):
        self.dt = dt   # 한 스텝 [s] — write · read 주기

    def ready(self) -> str | None:
        """움직여도 되는지 — 안 되면 이유 문자열 (real: MANUAL · /odom). 경고는 여기서 출력."""
        return None

    def start(self) -> None:
        """첫 write 직전 (real: 벽시계 기준점)."""

    def stop(self) -> None:
        """멈춤 (명령 0)."""
        self.write(0.0, 0.0)

    def close(self) -> None:
        """끝 — 자원 정리."""

    def write(self, v: float, w: float) -> None:
        """명령 (v [m/s], w [rad/s])을 보낸다."""
        raise NotImplementedError

    def pace(self, k: int) -> None:
        """k번째 스텝이 끝날 때까지 기다림 — real만 (sim은 물리 스텝이 시계, dummy는 계산만)."""

    def read(self) -> dict:
        """지금 상태 {STATE_KEYS} (+ real은 t_*)."""
        raise NotImplementedError
