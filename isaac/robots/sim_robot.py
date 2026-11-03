"""SimRobot — Isaac sim 로봇 (BaseEnv: 행동 [v, w] · env.sense()). 어떤 로봇이든 같다. Isaac 앱이 켜진 뒤 만든다."""
from __future__ import annotations

import math

from robots.robot_interface import RobotInterface


class SimRobot(RobotInterface):
    """write = env.step([[v, w]]) (보정 없음) · read = env.sense() — odom은 드라이버 /odom처럼 엔코더 적분. env 0번만."""

    def __init__(self, env):
        import torch

        super().__init__(env.policy_dt)
        self.torch, self.env = torch, env
        self.out, self.odom = (0.0, 0.0), [0.0, 0.0, 0.0]

    def write(self, v, w):
        self.out = (v, w)
        self.env.step(self.torch.tensor([[v, w]], device=self.env.device))

    def read(self):
        s, o, dt = self.env.sense(), self.odom, self.dt
        o[2] += s["w_enc"] * dt
        o[0] += s["v_enc"] * math.cos(o[2]) * dt
        o[1] += s["v_enc"] * math.sin(o[2]) * dt
        (wl, wr), (pl, pr) = s["jv"], s["jp"]
        return {"lin_out": self.out[0], "ang_out": self.out[1], "odom_x": o[0], "odom_y": o[1], "odom_yaw": o[2],
                "v_enc": s["v_enc"], "w_enc": s["w_enc"], "w_gyro": s["w_gyro"],
                "wheel_l_vel": wl, "wheel_r_vel": wr, "wheel_l_pos": pl, "wheel_r_pos": pr}
