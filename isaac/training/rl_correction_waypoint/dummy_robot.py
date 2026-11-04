"""DummyRobot — 이 로봇의 가짜 (Isaac/ROS 없이 calibration 흐름 확인). 바퀴 숫자 = 이 폴더 robot_cfg.py. 표준 라이브러리만.

모터: 드라이버 바퀴 식 → 명령 지연 → 좌/우 gain → 데드밴드 → 1차 지연 τ + 잡음 · 회전 슬립.
"""
from __future__ import annotations

import math
import os
import random
from collections import deque

import robots
from robots.robot_interface import RobotInterface

CONTROL_HZ = 20.0   # = envs/base_env_cfg.py CONTROL_HZ (Isaac 없이 돌아서 따로, 바꾸면 같이)


class DummyRobot(RobotInterface):
    def __init__(self, gain_lr=(0.99, 0.99), tau=0.05, delay=2, noise=0.002, slip=0.0, deadband=0.0, seed=0,
                 robot: str = os.path.dirname(os.path.abspath(__file__))):
        super().__init__(1.0 / CONTROL_HZ)
        c = robots.load(robot)
        self.r, self.sep, self.max_wheel = c.WHEEL_RADIUS, c.WHEEL_SEP, c.MAX_WHEEL
        self.gain_lr, self.tau, self.noise, self.slip, self.deadband = gain_lr, tau, noise, slip, deadband
        self.rng = random.Random(seed)
        self.buf = deque([(0.0, 0.0)] * (delay + 1), maxlen=delay + 1)
        self.plant, self.odom, self.out, self.k = (0.0, 0.0), [0.0, 0.0, 0.0], (0.0, 0.0), 0

    def write(self, v, w):
        self.out, self.k = (v, w), self.k + 1
        wl, wr = (v - w * self.sep / 2.0) / self.r, (v + w * self.sep / 2.0) / self.r
        k = min(1.0, self.max_wheel / max(abs(wl), abs(wr), 1e-9))
        self.buf.append((wl * k, wr * k))

    def read(self):
        (wl, wr), (gl, gr), dt = self.buf[0], self.gain_lr, self.dt
        tl, tr = [0.0 if abs(x) < self.deadband else x for x in (wl * gl, wr * gr)]
        a = 1.0 - math.exp(-dt / self.tau) if self.tau > 1e-9 else 1.0
        sl, sr = self.plant = tuple(s + a * (x - s) for s, x in zip(self.plant, (tl, tr)))
        n = lambda: self.rng.uniform(-self.noise, self.noise)  # noqa: E731
        v_enc, w_enc = self.r * (sl + sr) / 2.0 + n(), self.r * (sr - sl) / self.sep + n()
        w_gyro = self.r * (sr - sl) / self.sep * (1.0 - self.slip) + n()
        o = self.odom
        o[2] += w_enc * dt
        o[0] += v_enc * math.cos(o[2]) * dt
        o[1] += v_enc * math.sin(o[2]) * dt
        return {"lin_out": self.out[0], "ang_out": self.out[1], "odom_x": o[0], "odom_y": o[1], "odom_yaw": o[2],
                "v_enc": v_enc, "w_enc": w_enc, "w_gyro": w_gyro,
                "wheel_l_vel": sl, "wheel_r_vel": sr, "wheel_l_pos": 0.0, "wheel_r_pos": 0.0,
                **dict.fromkeys(("t_ros", "t_odom", "t_imu", "t_joint"), (self.k - 1) * dt)}   # 가짜 — 메시지 시각 = 계산 시각
