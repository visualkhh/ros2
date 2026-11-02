"""보정 실행기. 한 파일 한 클래스.

`python -m calibration <sensor-sync-sim|sensor-sync-real> [옵션...]` 으로 실행한다.
`Sensor` 하나만 받는다. segs는 호출자가 excitation으로 미리 만들어서 넘긴다
(조회 자체는 데이터만으로 됨 — 타입 의존 없음).
sim과 real 둘 다 이 클래스로 돈다 — 전송 방식 차이는 `Sensor` 서브클래스가 흡수하고,
real의 wall-clock pacing은 `pace` 훅으로 넣는다. sim+real 동시 실행·비교 묶음은
shell(`run-calibration.sh calibrate`)이 담당한다.
"""
from __future__ import annotations

import importlib
import importlib.util
import json
import os
import re
import sys
from collections.abc import Callable, Sequence
from datetime import datetime
from pathlib import Path

from calibration.parameter.SimParam import SimParam
from calibration.sensor.Sensor import CSV_HEADER, Sensor

ISAAC = Path(__file__).resolve().parents[1]
RUNS_DIR = ISAAC / "artifacts" / "calibration"


class Calibrate:
    """Sensor(수집)로 open-loop 기록 1회. segs는 호출자가 excitation으로 미리 만든다."""

    def __init__(self, sensor: Sensor):
        self.sensor = sensor

    def run(self, segs: list, total: float, out: str | Path | None, dt: float,
            pace: Callable[[int, float], None] | None = None,
            base_fit: dict | None = None, plant: dict | None = None) -> list:
        """같은 명령을 넣고 CSV로 저장. rows 반환.

        out이 None이면 artifacts/calibration/<stamp>-<run_name>/raw.csv에
        쌓는다 (run_name은 --label, 없으면 TAG. flat 파일은 호출자가 직접 지정).
        pace(k, t)가 있으면 send 뒤·collect 앞에 호출한다 (real wall-clock용).
        sim은 물리 스텝이 페이싱이라 pace 없이 돈다.
        base_fit(`load_fit` 결과)이 있으면 같은 폴더에 base-fit.json으로 남긴다.
        plant(측정 당시 sim 모터 설정 등)가 있으면 fit.json에 같이 적는다 — 학습이 같은 plant인지 확인용.
        """
        print(f"[{self.sensor.tag}] profile: {len(segs)} segs · {total:.1f}s")
        if out is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name = getattr(self.sensor, "run_name", self.sensor.tag)
            name = re.sub(r"[^0-9A-Za-z_.-]+", "_", str(name)).strip("._") or "run"
            out = str(RUNS_DIR / f"{stamp}-{name}" / "raw.csv")
        rows = []
        n = int(round(total / dt))
        for k in range(n):
            t = k * dt
            v, w = 0.0, 0.0
            for (_t0, t1, _v, _w, _label) in segs:
                if t < t1:
                    v, w = _v, _w
                    break
            self.sensor.send(v, w)
            if pace is not None:
                pace(k, t)
            rows.append(self.sensor.collect(t, v, w))
        os.makedirs(os.path.dirname(out), exist_ok=True)
        self.sensor.write_csv(out, rows)
        if base_fit is not None:
            base_path = os.path.join(os.path.dirname(out), "base-fit.json")
            with open(base_path, "w") as f:
                f.write(json.dumps({"source": base_fit["source"],
                                    "fit": {k: v for k, v in base_fit.items() if k != "source"}},
                                   indent=2, ensure_ascii=False) + "\n")
            print(f"[{self.sensor.tag}] base-fit.json → {base_path}")
        self.sensor.report(segs, rows, out)
        try:
            from calibration.report.report import fit_wheel, write_report_png
            recs = [dict(zip(CSV_HEADER, r)) for r in rows]
            fit = fit_wheel(recs)
            if fit is not None:
                fit_path = os.path.join(os.path.dirname(out), "fit.json")
                with open(fit_path, "w") as f:
                    doc = {"source": os.path.basename(out)}
                    if plant is not None:
                        doc["plant"] = plant
                    doc["fit"] = fit
                    f.write(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
                print(f"[{self.sensor.tag}] fit.json → {fit_path} "
                      f"(gain L/R={fit['wheel_gain_l']}/{fit['wheel_gain_r']} "
                      f"delay={fit['wheel_delay_steps']} tau={fit['wheel_tau_s']} "
                      f"db={fit['wheel_deadband']})")
            else:
                print(f"[{self.sensor.tag}] fit 스킵 (전진 구간 없음)")
            write_report_png(out, fit=fit)
        except Exception as exc:
            print(f"[{self.sensor.tag}] report.png/fit.json 스킵 ({exc})")
        return rows

    @staticmethod
    def load_fit(spec: str) -> dict:
        """`--fit` 값 → 입력 보정 (sim_calib 평탄 스키마, `SimParam.from_dict`가 읽음).

        `real/fit.json` 1개: 그 fit 그대로 (real plant 절대값).
        `real/fit.json,sim/fit.json` 2개: sim plant 위에 얹을 차이 —
        gain은 real/sim 비율, delay/tau는 real−sim (음수면 0).
        deadband는 빼기가 아니다: 보정단은 sim gain 앞에 있으므로 real 문턱을 sim gain으로
        나눈 값이 되고, sim 자체 문턱이 이미 더 크면 0 (데드밴드는 겹치면 큰 쪽만 남는다).
        """
        paths = [str(Path(p).expanduser().resolve()) for p in spec.split(",") if p.strip()]
        if len(paths) not in (1, 2):
            raise ValueError(f"--fit은 fit.json 1개 또는 'real,sim' 2개: {spec}")
        wheels = [SimParam.from_dict(json.loads(Path(p).read_text())["fit"]).wheel for p in paths]
        real = wheels[0]
        gl, gr = real.gains()
        delay, tau, db = real.delay_steps, real.tau_s, real.deadband
        if len(wheels) == 2:
            sim = wheels[1]
            sl, sr = sim.gains()
            # ponytail: 1차 지연 두 개 직렬 ≈ tau 합으로 근사, 정확히 하려면 2차 모델 fit
            gl, gr = gl / sl, gr / sr
            delay = max(0, delay - sim.delay_steps)
            tau = max(0.0, tau - sim.tau_s)
            db = db / ((sl + sr) / 2) if db > sim.deadband else 0.0
        return {"wheel_gain": round((gl + gr) / 2, 4),
                "wheel_gain_l": round(gl, 4), "wheel_gain_r": round(gr, 4),
                "wheel_delay_steps": int(delay), "wheel_tau_s": round(tau, 4),
                "wheel_deadband": round(db, 4), "source": paths}

    @staticmethod
    def _load_sensor_class(spec: str):
        """센서 클래스 로딩. `pkg.mod.Class` 또는 `path/to/file.py[:Class]`."""
        sensor_cls = Calibrate.load_class(spec, Sensor)
        if not callable(getattr(sensor_cls, "main", None)):
            raise AttributeError(f"{sensor_cls.__name__}에 main(argv) 클래스메서드 없음")
        return sensor_cls

    @staticmethod
    def load_excitation(spec: str | None):
        """`--excitation` 값 → 측정 경로 인스턴스 (없으면 DefaultCalibExcitation). 표준 라이브러리만 써서 sim 전에 불러도 된다."""
        from calibration.excitation.CalibExcitation import CalibExcitation
        from calibration.excitation.DefaultCalibExcitation import DefaultCalibExcitation
        return (Calibrate.load_class(spec, CalibExcitation) if spec else DefaultCalibExcitation)()

    @staticmethod
    def load_class(spec: str, base: type):
        """`pkg.mod.Class` 또는 `path/to/file.py[:Class]` → base의 서브클래스 (센서 · 장면 공용).

        파일만 주면 그 파일에 정의된 base 서브클래스가 1개여야 한다.
        """
        what = base.__name__
        path_part, _, class_name = spec.partition(":")
        if path_part.endswith(".py") or "/" in path_part or Path(path_part).exists():
            p = Path(path_part).expanduser()
            if not p.is_absolute():
                p = Path.cwd() / p
            p = p.resolve()
            if not p.is_file():
                raise FileNotFoundError(f"{what} 파일 없음: {p}")
            mod_name = f"calib_user_{abs(hash(str(p)))}"
            mod_spec = importlib.util.spec_from_file_location(mod_name, p)
            if mod_spec is None or mod_spec.loader is None:
                raise ImportError(f"{what} 로드 실패: {p}")
            mod = importlib.util.module_from_spec(mod_spec)
            sys.modules[mod_name] = mod
            mod_spec.loader.exec_module(mod)
            if class_name:
                found_cls = getattr(mod, class_name, None)
                if found_cls is None:
                    raise AttributeError(f"{p}에 {class_name} 없음")
            else:
                found = [v for v in vars(mod).values()
                         if isinstance(v, type) and issubclass(v, base)
                         and v is not base and v.__module__ == mod.__name__]
                if len(found) != 1:
                    raise AttributeError(
                        f"{p}에서 {what} 서브클래스를 1개로 특정 불가 ({len(found)}개). "
                        f"path/to/file.py:ClassName 형태로 지정하라")
                found_cls = found[0]
        else:
            if "." not in path_part:
                raise ValueError(f"{what} 지정은 'pkg.mod.Class' 또는 파일 경로: {spec}")
            mod_name, _, cls_name = path_part.rpartition(".")
            mod = importlib.import_module(mod_name)
            found_cls = getattr(mod, cls_name or class_name, None)
            if found_cls is None:
                raise AttributeError(f"{path_part}에 클래스 없음")
        if not (isinstance(found_cls, type) and issubclass(found_cls, base)):
            raise TypeError(f"{found_cls}는 {what} 서브클래스가 아님")
        return found_cls

    @classmethod
    def main(cls, argv: Sequence[str] | None = None) -> int:
        """시작점. `python -m calibration <센서> [옵션...]`.

        센서는 이름·dotted 경로·파일 경로로 지정한다. 모듈은 여기서 lazy import 한다 —
        sim 파일은 IsaacLab venv에서만, real 파일은 ROS 환경에서만 돌아가기 때문이다.
        """
        args = list(sys.argv[1:] if argv is None else argv)
        if not args or args[0] in ("-h", "--help", "help"):
            print("사용법: python -m calibration <센서> [옵션...]\n"
                  "  센서: sensor-sync-sim | sensor-sync-real\n"
                  "        | pkg.mod.Class | path/to/file.py[:Class]")
            return 0 if args else 2
        cmd, rest = args[0], args[1:]
        if cmd == "sensor-sync-sim":
            from calibration.sensor.DefaultSensorSim import DefaultSensorSim
            return DefaultSensorSim.main(rest) or 0
        if cmd == "sensor-sync-real":
            from calibration.sensor.DefaultSensorReal import DefaultSensorReal
            return DefaultSensorReal.main(rest) or 0
        try:
            sensor_cls = cls._load_sensor_class(cmd)
        except (FileNotFoundError, ImportError, AttributeError, TypeError, ValueError) as exc:
            print(f"센서 로드 실패: {exc}", file=sys.stderr)
            return 2
        return sensor_cls.main(rest) or 0


if __name__ == "__main__":
    raise SystemExit(Calibrate.main())
