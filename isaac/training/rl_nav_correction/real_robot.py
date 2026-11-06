"""RealRobot — 이 로봇(beebot)의 실기기. 명령은 PC에서 만들고, 재생과 기록은 로봇(bee)의 `run-cli calibrate wheel`가 한다.

beebot의 ROS는 로봇 안에서만 돈다 (DDS 공유 메모리, ROS_DOMAIN_ID=88) — PC에서 직접 토픽을 못 본다. 그래서:
  write  스텝(v, w)을 모은다 (실제로 보내지 않음) · pace는 기다리지 않음 (측정 전체가 data() 안에서 돈다)
  data   계획(JSON) → scp → ssh `run-cli calibrate wheel` (수동 경로 /cmd/manual/cmd_vel — 보정 없는 순수 응답,
         로봇이 20 Hz로 재생 · /odom /imu /joint_states /cmd_vel 기록) → raw.csv scp로 가져와 줄로 돌려줌
  ready  ssh `run-cli calibrate wheel --check-only` (안 움직임: MANUAL · E-STOP OFF · 토픽 도착)
준비: 로봇에서 bringup · MANUAL (`./run-cli mode manual`) · 사방 약 1.5 m 비움 (팔은 로봇이 재생 전에 stow로 자동 이동). 멈추려면 Ctrl+C (ssh -tt가 로봇에 전달) 또는 `./run-cli estop on`.
설정(환경변수): BEE_HOST (bee@bee.local) · BEE_TOOL (로봇의 tool 폴더) · BEE_ENV (로봇 ROS 환경 — 기본 도메인 88 · 로컬 DDS)
표준 라이브러리만 (ssh · scp 필요).
"""
from __future__ import annotations

import csv
import json
import os
import subprocess
import tempfile
import time

from robots.robot_interface import RobotInterface

HOST = os.environ.get("BEE_HOST", "bee@bee.local")
TOOL = os.environ.get("BEE_TOOL", "/home/khh/Lv2_bee_Assignment/lv2_module6/tool")
ENV = os.environ.get("BEE_ENV", "ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET "
                                "FASTDDS_DEFAULT_PROFILES_FILE=/home/bee/Lv2_bee_Assignment/lv2_module6/ros2_ws/fastdds_local.xml")
SSH = ["ssh", "-o", "ConnectTimeout=8", "-o", "BatchMode=yes"]


class RealRobot(RobotInterface):
    def __init__(self, rate: float = 20.0):
        super().__init__(1.0 / rate)
        self.steps = []
        self.stamp = time.strftime("%Y%m%d_%H%M%S")

    def write(self, v, w):
        if self.rows is not None:   # 측정 중에만 계획에 쌓는다 (stop()의 0 명령은 안 쌓음 — 아래 stop)
            self.steps.append([float(v), float(w), ""])

    def start(self):
        super().start()
        self.steps = []

    def stop(self):
        """측정은 로봇이 끝내며 스스로 정지한다 — PC에서 보낼 것 없음."""

    def _plan_file(self, steps) -> str:
        f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump({"dt": self.dt, "label": "real", "steps": steps}, f)
        f.close()
        return f.name

    def _run(self, remote: str, tty: bool = False) -> int:
        """로봇에서 명령 하나 (ROS 환경 포함). tty = Ctrl+C가 로봇까지 가게 (-tt)."""
        return subprocess.run(SSH + (["-tt"] if tty else []) + [HOST, f"export {ENV}; cd {TOOL} && {remote}"]).returncode

    def _send(self, plan: str) -> str:
        remote = f"/tmp/calib_plan_{self.stamp}.json"
        subprocess.run(["scp", "-q", "-o", "BatchMode=yes", plan, f"{HOST}:{remote}"], check=True)
        return remote

    def ready(self):
        plan = self._plan_file([[0.0, 0.0, ""]])
        try:
            remote = self._send(plan)
            rc = self._run(f"./run-cli calibrate wheel {remote} --check-only")
        except (subprocess.CalledProcessError, OSError) as exc:
            return f"{HOST} 접속 실패: {exc} — 같은 네트워크 · ssh 키 확인"
        finally:
            os.unlink(plan)
        return None if rc == 0 else "로봇 준비 안 됨 (위 출력 — MANUAL · E-STOP · bringup)"

    def data(self):
        steps, self.rows = self.steps, None
        plan = self._plan_file(steps)
        out_dir = f"/tmp/calib_{self.stamp}"
        local = tempfile.mkdtemp()
        try:
            remote = self._send(plan)
            print(f"[real] {HOST} 에서 재생 시작 — {len(steps)}스텝 · {len(steps) * self.dt:.0f}s (Ctrl+C = 정지)")
            rc = self._run(f"./run-cli calibrate wheel {remote} --out {out_dir}/raw.csv", tty=True)
            if rc != 0:
                raise SystemExit(f"[real] 로봇 측정 실패 (rc={rc}) — 위 출력 확인")
            subprocess.run(["scp", "-q", "-o", "BatchMode=yes", f"{HOST}:{out_dir}/raw.csv", local + "/raw.csv"], check=True)
        finally:
            os.unlink(plan)
        with open(local + "/raw.csv", newline="") as f:
            return [{k: (v if k == "label" else float(v)) for k, v in row.items()} for row in csv.DictReader(f)]
