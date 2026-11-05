"""SimRobot — 이 로봇의 sim (Isaac, BaseEnv: 행동 [v, w] · env.sense()). real_robot.py · dummy_robot.py와 짝. Isaac 앱이 켜진 뒤 만든다."""
from __future__ import annotations

import math

from robots.robot_interface import RobotInterface


class SimRobot(RobotInterface):
    """write = env.step([[v, w]]) (보정 없음) + 측정 중이면 env.sense() 한 줄을 쌓음 — odom은 드라이버 /odom처럼 엔코더 적분. env 0번만."""

    def __init__(self, env):
        import torch

        super().__init__(env.policy_dt)
        self.torch, self.env = torch, env
        self.out, self.odom, self.k = (0.0, 0.0), [0.0, 0.0, 0.0], 0

    def start(self):
        super().start()
        self.k, self.odom = 0, [0.0, 0.0, 0.0]   # 측정 시작 = t 0 · odom 원점

    def write(self, v, w):
        self.out, self.k = (v, w), self.k + 1
        self.env.step(self.torch.tensor([[v, w]], device=self.env.device))
        if self.rows is not None:   # 측정 중 — 이 스텝 뒤 상태를 쌓는다
            self.rows.append(self._state())

    def close(self):
        self.env.close()

    def _state(self):
        s, o, dt = self.env.sense(), self.odom, self.dt
        o[2] += s["w_enc"] * dt
        o[0] += s["v_enc"] * math.cos(o[2]) * dt
        o[1] += s["v_enc"] * math.sin(o[2]) * dt
        (wl, wr), (pl, pr) = s["jv"], s["jp"]
        t = (self.k - 1) * dt
        return {"t": round(t, 3), "lin_raw": self.out[0], "ang_raw": self.out[1],
                "lin_out": self.out[0], "ang_out": self.out[1], "odom_x": o[0], "odom_y": o[1], "odom_yaw": o[2],
                "v_enc": s["v_enc"], "w_enc": s["w_enc"], "w_gyro": s["w_gyro"],
                "wheel_l_vel": wl, "wheel_r_vel": wr, "wheel_l_pos": pl, "wheel_r_pos": pr,
                **dict.fromkeys(("t_ros", "t_odom", "t_imu", "t_joint"), t)}   # sim은 시계 하나 — 메시지 시각 = sim 시각
