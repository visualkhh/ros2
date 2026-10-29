"""Before/After 비교 — eval_recorder CSV 두 개 → 트래킹 지표 (documents/isaac/task-spec.md §4).

  python3 compare_csv.py /tmp/before.csv /tmp/after.csv

명령(lin_raw/ang_raw = 보정 전, 사람·Nav2가 원한 것) vs 실제(v_enc, w_gyro). 정지 명령 행은 뺌.
"""
import csv
import math
import sys


def metrics(path: str) -> dict:
    rows = [r for r in csv.DictReader(open(path)) if float(r['lin_raw']) != 0.0 or float(r['ang_raw']) != 0.0]
    if not rows:
        return {'rows': 0}
    def rms(key_cmd, key_real):
        return math.sqrt(sum((float(r[key_cmd]) - float(r[key_real])) ** 2 for r in rows) / len(rows))
    return {'rows': len(rows),
            'rms_v (m/s)': rms('lin_raw', 'v_enc'),
            'rms_w_gyro (rad/s)': rms('ang_raw', 'w_gyro'),
            'rms_w_enc (rad/s)': rms('ang_raw', 'w_enc'),
            'mean |corr_w|': sum(abs(float(r['ang_out']) - float(r['ang_raw'])) for r in rows) / len(rows)}


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    before, after = metrics(sys.argv[1]), metrics(sys.argv[2])
    print(f"{'':22s}{'before':>10s}{'after':>10s}{'change':>10s}")
    for k in before:
        b, a = before[k], after.get(k, 0)
        ch = f'{(a - b) / b * 100:+.0f}%' if k != 'rows' and b else ''
        print(f'{k:22s}{b:10.4g}{a:10.4g}{ch:>10s}')


if __name__ == '__main__':
    main()
