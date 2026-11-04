"""RobotInterface — sim · real · 가짜 로봇을 같은 방식으로: write(v, w)로 명령을 보내고, 측정 중 쌓인 기록은 data()로 한꺼번에.

ros2_control hardware_interface의 write()와 같은 역할 — 추상 클래스, write는 로봇 폴더가 구현한다.
측정 중에는 로봇이 스스로 기록을 쌓는다 (언제 · 무엇을 쌓을지는 로봇이 정함 — self.rows.append(dict)). 끝나면 data()로 한꺼번에.
  <로봇 폴더>/sim_robot.py    SimRobot     Isaac env (env.step · env.sense)
  <로봇 폴더>/real_robot.py   RealRobot    그 로봇의 ROS 토픽
  <로봇 폴더>/dummy_robot.py  DummyRobot   가짜 모터 (표준 라이브러리만)
  (로봇 폴더 = training/<task>/, robots.load(robot, "sim_robot" | "real_robot" | "dummy_robot")로 읽는다)
이 파일은 표준 라이브러리만.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

# 쌓는 dict의 키가 곧 calibration CSV 열이다 (로봇마다 다를 수 있음). 지금 로봇들(차동 구동)은
#   t (기록 시각 [s]) · lin_raw · ang_raw (보낸 명령) · lin_out · ang_out (실제로 나간 명령 [m/s · rad/s]) · odom_x · odom_y · odom_yaw (엔코더 적분 자세) · v_enc · w_enc (엔코더 속도)
#   · w_gyro [rad/s] · wheel_l/r_vel [rad/s] · wheel_l/r_pos [rad] · t_ros · t_odom · t_imu · t_joint (메시지 시각 [s], sim · dummy는 자기 시각)
# calibration report(fit · 그림)는 이 이름들을 쓴다 — 없는 열은 건너뛴다.


class RobotInterface(ABC):
    def __init__(self, dt: float):
        self.dt = dt       # 한 스텝 [s] — write 주기
        self.rows = None   # 측정 중 쌓이는 기록 (start ~ data 사이에만 list)

    def ready(self) -> str | None:
        """움직여도 되는지 — 안 되면 이유 문자열 (real: MANUAL · /odom). 경고는 여기서 출력."""
        return None

    def start(self) -> None:
        """측정 시작 — 기록을 비우고 쌓기 시작 (real은 벽시계 기준점도)."""
        self.rows = []

    def data(self) -> list[dict]:
        """측정 끝 — 쌓인 기록 전부 (한 줄 = dict). 이후로는 안 쌓는다."""
        rows, self.rows = self.rows or [], None
        return rows

    def stop(self) -> None:
        """멈춤 (명령 0)."""
        self.write(0.0, 0.0)

    def close(self) -> None:
        """끝 — 자원 정리."""

    @abstractmethod
    def write(self, v: float, w: float) -> None:
        """명령 (v [m/s], w [rad/s])을 보낸다 — 측정 중(self.rows가 list)이면 그 스텝의 상태를 쌓는다."""

    def pace(self, k: int) -> None:
        """k번째 스텝이 끝날 때까지 기다림 — real만 (sim은 물리 스텝이 시계, dummy는 계산만)."""
