"""sensor_sync sim/real 공용 — 수집 계약 + CSV 스키마 + 구간 리포트.

sim(`DefaultSensorSim`, IsaacLab venv)과 real(`DefaultSensorReal`, ROS)은
런타임이 달라 실행 루프를 합칠 수 없다. 수집(send/collect)은 추상으로 두고,
CSV 한 줄 포맷·구간 평균·콘솔 표만 여기서 공유한다.
"""
from __future__ import annotations

import csv
from abc import ABC, abstractmethod

CSV_HEADER = ["t", "lin_raw", "ang_raw", "lin_out", "ang_out",
              "odom_x", "odom_y", "odom_yaw", "v_enc", "w_enc", "w_gyro",
              "wheel_l_vel", "wheel_r_vel", "wheel_l_pos", "wheel_r_pos", "label",
              "t_ros", "t_odom", "t_imu", "t_joint"]


class Sensor(ABC):
    """수집 계약. send/collect는 전송 방식별로 서브클래스가 구현한다."""

    TAG = "sync"

    def __init__(self, tag: str | None = None):
        self.tag = tag or self.TAG
        self.run_name = self.tag   # 기본 폴더 이름 — --label을 주면 서브클래스가 바꾼다

    @abstractmethod
    def send(self, v: float, w: float) -> None:
        """명령 전송. sim=관절 타깃+물리 스텝, real=Twist 발행."""
        raise NotImplementedError

    @abstractmethod
    def collect(self, t: float, v: float, w: float) -> list:
        """현재 센서값으로 CSV 한 행. sim=시뮬 상태 적분, real=토픽 스냅샷."""
        raise NotImplementedError

    @staticmethod
    def average(rows, t0: float, t1: float, skip: float = 0.0):
        """[t0+skip, t1) 구간 (v_enc, w_enc, w_gyro, wl, wr) 평균. 비면 [t0, t1)로 재시도."""
        part = [r for r in rows if t0 + skip <= r[0] < t1]
        if not part:
            part = [r for r in rows if t0 <= r[0] < t1]
        return tuple(sum(r[i] for r in part) / len(part) for i in (8, 9, 10, 11, 12))

    @staticmethod
    def find(segs, label: str):
        return next(s for s in segs if s[4] == label)

    @staticmethod
    def move_speeds_text(segs) -> str:
        return " / ".join(f"{s[2]:.2f}" for s in segs if s[4] == "move_left")

    @staticmethod
    def write_csv(out: str, rows) -> None:
        with open(out, "w", newline="") as f:
            cw = csv.writer(f)
            cw.writerow(CSV_HEADER)
            cw.writerows(rows)

    def report(self, segs, rows, out: str) -> None:
        """구간별 move/turn 표 — sim/real 출력 포맷 동일, 접두사만 tag로 다름."""
        for name in ("left", "right"):
            moves = [s for s in segs if s[4] == f"move_{name}"]
            for i, move in enumerate(moves, 1):
                vf, wf, gf, lf, rf = self.average(rows, move[0], move[1], skip=0.5)
                gain = vf / move[2] if move[2] else 0.0
                print(f"[{self.tag}] {name.upper():5s} move{i} cmd={move[2]:.3f} "
                      f"v_enc={vf:+.3f} gain={gain:.3f} w_enc={wf:+.3f} "
                      f"w_gyro={gf:+.3f} | 바퀴 wl={lf:+.2f} wr={rf:+.2f} rad/s")
            turn = self.find(segs, f"turn_{name}")
            vt, wt, gt, lt, rt = self.average(rows, turn[0], turn[1], skip=0.3)
            print(f"[{self.tag}] {name.upper():5s} turn  v_enc={vt:+.3f} w_enc={wt:+.3f} "
                  f"w_gyro={gt:+.3f} | 바퀴 wl={lt:+.2f} wr={rt:+.2f} rad/s")
        print(f"[{self.tag}] → {out}")
