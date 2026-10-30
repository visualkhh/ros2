"""S5 — 계단응답 CSV 비교 (sim ↔ real, 또는 before ↔ after).

  python3 steps/compare_step.py steps/artifacts/sim_step.csv /tmp/real_step.csv
  python3 steps/compare_step.py /tmp/before.csv /tmp/after.csv

같은 포맷(eval_recorder / s05_step_response.py):
  t, lin_raw, ang_raw, lin_out, ang_out, odom_x, odom_y, odom_yaw, v_enc, w_enc, w_gyro, label

지표:
  rms_v     = RMS(lin_raw − v_enc)          (움직이는 행만)
  rms_w     = RMS(ang_raw − w_gyro)         (자이로 기준)
  bias      = 평균 부호 오차 (명령 − 실제)
  p95       = |오차| 95퍼센타일
  t10/t90   = 첫 전진·회전 계단에서 10%·90% 도달 시간 [s]
  seg       = 명령이 일정한 구간별 평균
"""
import csv
import math
import sys


def load(path: str):
    rows = []
    for r in csv.DictReader(open(path)):
        rows.append({k: (v if k == "label" else float(v)) for k, v in r.items()})
    if not rows:
        raise SystemExit(f"빈 CSV: {path}")
    t0 = rows[0]["t"]
    for r in rows:
        r["t"] -= t0
    return rows


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def percentile(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))]


def first_step(rows, cmd_key, resp_key, frac):
    """첫 계단(0 → nonzero) 후 resp가 frac·cmd에 도달하는 시간. 없으면 nan."""
    for i in range(1, len(rows)):
        target = rows[i][cmd_key]
        prev = rows[i - 1][cmd_key]
        if abs(target) < 1e-6 or abs(prev - target) < 1e-6:
            continue
        goal = frac * target
        for j in range(i, len(rows)):
            if (target > 0 and rows[j][resp_key] >= goal) or (target < 0 and rows[j][resp_key] <= goal):
                return rows[j]["t"] - rows[i]["t"]
        return float("nan")
    return float("nan")


def segments(rows, min_s=0.4):
    """명령(lin_raw, ang_raw)이 일정한 구간. (start, end, v, w, mean_v, mean_w)"""
    out = []
    start = 0
    for i in range(1, len(rows) + 1):
        same = i < len(rows) and round(rows[i]["lin_raw"], 3) == round(rows[start]["lin_raw"], 3) \
            and round(rows[i]["ang_raw"], 3) == round(rows[start]["ang_raw"], 3)
        if same:
            continue
        if rows[i - 1]["t"] - rows[start]["t"] >= min_s:
            part = rows[start:i]
            out.append((part[0]["t"], part[-1]["t"], part[0]["lin_raw"], part[0]["ang_raw"],
                        sum(r["v_enc"] for r in part) / len(part),
                        sum(r["w_gyro"] for r in part) / len(part)))
        start = i
    return out


def metrics(rows):
    moving = [r for r in rows if abs(r["lin_raw"]) > 1e-9 or abs(r["ang_raw"]) > 1e-9]
    ev = [r["lin_raw"] - r["v_enc"] for r in moving]
    ew = [r["ang_raw"] - r["w_gyro"] for r in moving]
    ew_enc = [r["ang_raw"] - r["w_enc"] for r in moving]
    return {
        "rows": len(rows),
        "rms_v": rms(ev),
        "rms_w": rms(ew),
        "rms_w_enc": rms(ew_enc),
        "bias_v": sum(ev) / len(ev) if ev else float("nan"),
        "bias_w": sum(ew) / len(ew) if ew else float("nan"),
        "bias_w_enc": sum(ew_enc) / len(ew_enc) if ew_enc else float("nan"),
        "p95_v": percentile([abs(x) for x in ev], 0.95),
        "p95_w": percentile([abs(x) for x in ew], 0.95),
        "t10_v": first_step(rows, "lin_raw", "v_enc", 0.10),
        "t90_v": first_step(rows, "lin_raw", "v_enc", 0.90),
        "t10_w": first_step(rows, "ang_raw", "w_gyro", 0.10),
        "t90_w": first_step(rows, "ang_raw", "w_gyro", 0.90),
    }


def fmt(x, nd=3):
    return "nan" if isinstance(x, float) and math.isnan(x) else f"{x:.{nd}f}"


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit(__doc__)
    names = ["sim" if len(sys.argv) == 2 else "A", "real" if len(sys.argv) == 2 else "B"]
    data = [metrics(load(p)) for p in sys.argv[1:]]
    keys = ["rows", "rms_v", "rms_w", "rms_w_enc", "bias_v", "bias_w", "bias_w_enc",
            "p95_v", "p95_w", "t10_v", "t90_v", "t10_w", "t90_w"]
    print(f"{'metric':12s}" + "".join(f"{n:>12s}" for n in names[:len(data)]) + ("     diff" if len(data) == 2 else ""))
    for k in keys:
        vals = [d[k] for d in data]
        line = f"{k:12s}" + "".join(f"{fmt(v):>12s}" for v in vals)
        if len(data) == 2 and k != "rows" and vals[0] and not (math.isnan(vals[0]) or math.isnan(vals[1])):
            line += f"  {(vals[1] - vals[0]) / abs(vals[0]) * 100:+6.0f}%"
        elif len(data) == 2 and k != "rows":
            line += "       —"
        print(line)

    for p, d, name in zip(sys.argv[1:], data, names[:len(data)]):
        rows = load(p)
        print(f"\n[{name}] {p}")
        for start, end, v, w, mv, mw in segments(rows):
            print(f"  {start:5.1f}~{end:5.1f}s  cmd v={v:+.2f} w={w:+.2f} | "
                  f"mean v_enc={mv:+.3f} w_gyro={mw:+.3f} | err v={v - mv:+.3f} w={w - mw:+.3f}")


if __name__ == "__main__":
    main()
