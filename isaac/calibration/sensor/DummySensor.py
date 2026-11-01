"""가짜 센서 — Isaac/ROS 없이 calibration 파이프라인을 돌려볼 때 쓴다.

1차 지연 + gain + 잡음으로 바퀴 응답을 흉내 내고, odom은 v_enc 적분으로 만든다
(sim과 같은 식). 출력 CSV 스키마는 실측과 동일.

실행 (isaaclab/ROS 없어도 됨):
  python -m calibration /path/to/DummySensor.py --side 0.5 --out /tmp/dummy.csv
  python /path/to/DummySensor.py --side 0.5 --out /tmp/dummy.csv
  # 입력 보정 (sim의 WheelLag 자리): real/fit.json 또는 real/fit.json,sim/fit.json
  python -m calibration calibration/sensor/DummySensor.py --gain 1.0 --fit real/fit.json,sim/fit.json
"""
from __future__ import annotations

import argparse
import math
import random
import sys
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ISAAC))

from calibration.Calibrate import Calibrate  # noqa: E402
from calibration.excitation.DefaultCalibExcitation import DefaultCalibExcitation  # noqa: E402
from calibration.parameter.SimParam import SimParam  # noqa: E402
from calibration.parameter.WheelParam import WheelParam  # noqa: E402
from calibration.sensor.Sensor import Sensor  # noqa: E402


class DummySensor(Sensor):
    """stdlib만 쓰는 가짜 센서. send=입력 보정+지연 버퍼, collect=1차 지연+잡음+odom 적분.

    입력 보정(`base`)은 sim의 WheelLag 자리 — torch가 없어 같은 순서를 여기서 돈다.
    """

    TAG = "dummy"

    def __init__(self, label: str = "dummy", gain: float = 0.99,
                 gain_l: float | None = None, gain_r: float | None = None,
                 tau_s: float = 0.05, delay_steps: int = 2,
                 noise: float = 0.002, slip: float = 0.0,
                 deadband: float = 0.0, seed: int = 0, base: dict | None = None):
        super().__init__()
        self.label = label
        self.gain = gain
        self.gain_l = gain if gain_l is None else gain_l
        self.gain_r = gain if gain_r is None else gain_r
        self.slip = slip
        self.deadband = float(deadband)
        self.tau_s = tau_s
        self.delay_steps = max(0, int(delay_steps))
        self.noise = noise
        self._rng = random.Random(seed)
        self._buf = deque([(0.0, 0.0)] * (self.delay_steps + 1),
                          maxlen=self.delay_steps + 1)
        self._state = (0.0, 0.0)
        self.base = WheelParam(tau_s=0.0, delay_steps=0, gain=1.0) if base is None \
            else SimParam.from_dict(base).wheel
        self._base_buf = deque([(0.0, 0.0)] * (self.base.delay_steps + 1),
                               maxlen=self.base.delay_steps + 1)
        self._base_state = (0.0, 0.0)
        self.ox = self.oy = self.oyaw = 0.0
        self.wheel_l = self.wheel_r = 0.0

    def send(self, v: float, w: float) -> None:
        # 드라이버와 같은 식: v,w → 좌/우 rad/s → 8 rad/s 비례 축소
        wl = (v - w * 0.287 / 2.0) / 0.033
        wr = (v + w * 0.287 / 2.0) / 0.033
        peak = max(abs(wl), abs(wr))
        if peak > 8.0:
            k = 8.0 / peak
            wl, wr = wl * k, wr * k
        # 입력 보정 — WheelLag와 같은 순서: delay → gain → deadband → 1차 지연
        self._base_buf.append((wl, wr))
        wl, wr = self._base_buf[0]
        gl, gr = self.base.gains()
        wl, wr = wl * gl, wr * gr
        if abs(wl) < self.base.deadband:
            wl = 0.0
        if abs(wr) < self.base.deadband:
            wr = 0.0
        if self.base.tau_s > 1e-6:
            a = 1.0 - math.exp(-0.05 / self.base.tau_s)
            bl, br = self._base_state
            wl, wr = bl + a * (wl - bl), br + a * (wr - br)
        self._base_state = (wl, wr)
        self._buf.append((wl, wr))

    def collect(self, t: float, v: float, w: float) -> list:
        # plant: 지연 → gain → 데드밴드 → 1차 지연
        wl, wr = self._buf[0]
        target_l = wl * self.gain_l
        target_r = wr * self.gain_r
        if abs(target_l) < self.deadband:
            target_l = 0.0
        if abs(target_r) < self.deadband:
            target_r = 0.0
        alpha = 1.0 - math.exp(-0.05 / self.tau_s) if self.tau_s > 1e-9 else 1.0
        sl, sr = self._state
        sl += alpha * (target_l - sl)
        sr += alpha * (target_r - sr)
        self._state = (sl, sr)
        self.wheel_l, self.wheel_r = sl, sr
        v_enc = 0.033 * (sl + sr) / 2.0 + self._rng.uniform(-self.noise, self.noise)
        w_enc = 0.033 * (sr - sl) / 0.287 + self._rng.uniform(-self.noise, self.noise)
        body_w = 0.033 * (sr - sl) / 0.287 * (1.0 - self.slip)
        w_gyro = body_w + self._rng.uniform(-self.noise, self.noise)
        self.oyaw += w_enc * 0.05
        self.ox += v_enc * math.cos(self.oyaw) * 0.05
        self.oy += v_enc * math.sin(self.oyaw) * 0.05
        return [round(t, 3), v, w, v, w, self.ox, self.oy, self.oyaw,
                v_enc, w_enc, w_gyro, self.wheel_l, self.wheel_r,
                0.0, 0.0, self.label,
                t, t, t, t]

    @classmethod
    def main(cls, argv=None):
        parser = argparse.ArgumentParser(description="dummy 센서로 calibration 한 바퀴")
        parser.add_argument("--side", type=float, default=0.5)
        parser.add_argument("--gain", type=float, default=0.99, help="모터 gain (1.0=이상, 낮추면 저보상 sim)")
        parser.add_argument("--gain-l", type=float, default=None, help="왼쪽 gain (기본: --gain)")
        parser.add_argument("--gain-r", type=float, default=None, help="오른쪽 gain (기본: --gain)")
        parser.add_argument("--noise", type=float, default=0.002, help="센서 노이즈 크기")
        parser.add_argument("--slip", type=float, default=0.0,
                            help="회전 슬립 비율 (0=무슬립, 0.1=실제 몸체 회전 10%% 손실)")
        parser.add_argument("--deadband", type=float, default=0.0,
                            help="바퀴 데드밴드 [rad/s] (이보다 작은 명령은 안 돎)")
        parser.add_argument("--speed", type=float, default=0.15)
        parser.add_argument("--turn", type=float, default=1.0)
        parser.add_argument("--out", default=None, help="기본: 자동 artifacts/<시각>-<TAG>/raw.csv")
        parser.add_argument("--label", default="dummy")
        parser.add_argument("--seed", type=int, default=0)
        parser.add_argument("--fit", default=None,
                            help="입력 보정: real/fit.json 또는 real/fit.json,sim/fit.json (차이)")
        args = parser.parse_args(argv)

        base_fit = Calibrate.load_fit(args.fit) if args.fit else None
        excitation = DefaultCalibExcitation()
        segs = excitation.make_profile(args.speed, args.turn, args.side)
        total = excitation.total(segs)
        sensor = cls(label=args.label, seed=args.seed, gain=args.gain,
                     gain_l=args.gain_l, gain_r=args.gain_r,
                     noise=args.noise, slip=args.slip, deadband=args.deadband,
                     base=base_fit)
        cal = Calibrate(sensor)
        cal.run(segs, total, args.out, 0.05, base_fit=base_fit)
        return 0


if __name__ == "__main__":
    raise SystemExit(DummySensor.main())
