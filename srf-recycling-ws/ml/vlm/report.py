#!/usr/bin/env python3
"""학습 보고서 (YOLO 의 results.csv / results.png 처럼): RUN/metrics.csv → RUN/report.html.

  train.py 가 기록할 때마다 자동으로 다시 쓴다. 브라우저로 열어 두면 30초마다 새로 고친다.
  ml/vlm/.venv/bin/python ml/vlm/report.py RUN                      # 다시 그리기
  ml/vlm/.venv/bin/python ml/vlm/report.py RUN --from-log train.log  # 콘솔 로그로 metrics.csv 만들기 (이전 실행용)
"""
import argparse
import csv
import html
import re
import time
from pathlib import Path

FIELDS = ["time", "epoch", "epochs", "sample", "samples", "loss", "lr", "sec_per_sample", "eta_min",
          "val_loss", "val_decision", "val_json"]


def append(run, row):
    path = Path(run) / "metrics.csv"
    new = not path.exists()
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in FIELDS})


def read(run):
    path = Path(run) / "metrics.csv"
    return list(csv.DictReader(path.open())) if path.exists() else []


def svg_chart(title, points, color, y_label, height=220, width=640):
    """points: [(x, y)]. 축·눈금이 있는 꺾은선 (의존성 없이 SVG)."""
    if not points:
        return f"<div class='chart empty'><h3>{html.escape(title)}</h3><p>아직 값이 없다</p></div>"
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x0, x1 = min(xs), max(xs) if max(xs) > min(xs) else min(xs) + 1
    y0, y1 = min(0.0, min(ys)), max(ys) * 1.05 if max(ys) > 0 else 1.0
    left, right, top, bottom = 64, 12, 10, 28
    w, h = width - left - right, height - top - bottom
    sx = lambda x: left + (x - x0) / (x1 - x0) * w  # noqa: E731
    sy = lambda y: top + h - (y - y0) / (y1 - y0) * h  # noqa: E731
    path = " ".join(f"{'M' if i == 0 else 'L'}{sx(x):.1f},{sy(y):.1f}" for i, (x, y) in enumerate(points))
    dots = "".join(f"<circle cx='{sx(x):.1f}' cy='{sy(y):.1f}' r='3' fill='{color}'/>" for x, y in points) \
        if len(points) <= 30 else ""
    grid = ""
    for k in range(5):
        v = y0 + (y1 - y0) * k / 4
        grid += (f"<line x1='{left}' x2='{left + w}' y1='{sy(v):.1f}' y2='{sy(v):.1f}' class='grid'/>"
                 f"<text x='{left - 6}' y='{sy(v) + 4:.1f}' text-anchor='end'>{v:.3g}</text>")
    for k in range(5):
        v = x0 + (x1 - x0) * k / 4
        grid += f"<text x='{sx(v):.1f}' y='{height - 8}' text-anchor='middle'>{v:.3g}</text>"
    last = ys[-1]
    return (f"<div class='chart'><h3>{html.escape(title)} <span>{last:.4g}</span></h3>"
            f"<svg viewBox='0 0 {width} {height}' role='img' aria-label='{html.escape(title)}'>{grid}"
            f"<path d='{path}' fill='none' stroke='{color}' stroke-width='2'/>{dots}"
            f"<text x='{left}' y='{top + 10}' class='unit'>{html.escape(y_label)}</text></svg></div>")


def write_report(run, title=None):
    run = Path(run)
    rows = read(run)
    num = lambda r, k: float(r[k]) if r.get(k) not in (None, "") else None  # noqa: E731
    progress = [(num(r, "epoch") - 1 + num(r, "sample") / num(r, "samples"), r) for r in rows if r.get("sample")]
    loss = [(x, num(r, "loss")) for x, r in progress if num(r, "loss") is not None]
    speed = [(x, num(r, "sec_per_sample")) for x, r in progress if num(r, "sec_per_sample") is not None]
    lr = [(x, num(r, "lr")) for x, r in progress if num(r, "lr") is not None]
    val = [(num(r, "epoch"), r) for r in rows if r.get("val_loss")]
    val_loss = [(e, num(r, "val_loss")) for e, r in val]
    val_dec = [(e, num(r, "val_decision")) for e, r in val if r.get("val_decision")]
    last = progress[-1][1] if progress else {}
    done = bool(last) and last.get("epoch") == last.get("epochs") and last.get("sample") == last.get("samples")
    pct = 100.0 * progress[-1][0] / float(last["epochs"]) if progress else 0.0
    eta = last.get("eta_min") or "-"
    config = (run / "config.json").read_text() if (run / "config.json").exists() else "{}"
    status = "끝" if done else f"진행 중 {pct:.1f}% · 남은 시간 약 {eta}분"
    page = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
{'' if done else '<meta http-equiv="refresh" content="30">'}
<title>{html.escape(title or 'VLM 학습 ' + run.name)}</title>
<style>
:root{{--bg:#fafaf9;--fg:#1c1917;--muted:#78716c;--card:#fff;--line:#e7e5e4}}
@media (prefers-color-scheme:dark){{:root{{--bg:#1c1917;--fg:#f5f5f4;--muted:#a8a29e;--card:#292524;--line:#44403c}}}}
body{{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,sans-serif}}
main{{max-width:1340px;margin:auto}} h1{{font-size:20px;margin:0 0 4px}} .muted{{color:var(--muted)}}
.bar{{height:10px;background:var(--line);border-radius:5px;overflow:hidden;margin:10px 0 16px}}
.bar>div{{height:100%;width:{pct:.1f}%;background:#2563eb}}
.grid2{{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px}}
.chart{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 12px}}
.chart h3{{margin:0 0 6px;font-size:14px;font-weight:600}} .chart h3 span{{float:right;color:var(--muted);font-weight:500}}
svg{{width:100%;height:auto}} svg text{{fill:var(--muted);font-size:11px}} svg .grid{{stroke:var(--line)}}
pre{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px;overflow:auto;font-size:12px}}
</style></head><body><main>
<h1>{html.escape(title or 'VLM 학습 · ' + run.name)}</h1>
<div class="muted">{status} · 에폭 {last.get('epoch', '-')}/{last.get('epochs', '-')} · 샘플 {last.get('sample', '-')}/{last.get('samples', '-')} · 갱신 {time.strftime('%H:%M:%S')}</div>
<div class="bar"><div></div></div>
<div class="grid2">
{svg_chart('학습 손실 (누적 평균)', loss, '#2563eb', 'loss')}
{svg_chart('검증 손실 (에폭마다)', val_loss, '#dc2626', 'val loss')}
{svg_chart('검증 판단 정답률 (select/hold)', val_dec, '#16a34a', '0~1')}
{svg_chart('학습률', lr, '#9333ea', 'lr')}
{svg_chart('속도', speed, '#ea580c', '초/샘플')}
</div>
<h3>설정</h3><pre>{html.escape(config)}</pre>
<p class="muted">x축 = 에폭 (1.5 = 2번째 에폭 절반). 원본 값: metrics.csv</p>
</main></body></html>"""
    (run / "report.html").write_text(page)


LOG_LINE = re.compile(r"\[train\] epoch (\d+)/(\d+) (\d+)/(\d+) loss ([\d.]+) \(([\d.]+)s")


def from_log(run, log):
    path = Path(run) / "metrics.csv"
    if path.exists():
        path.unlink()
    for line in Path(log).read_text().splitlines():
        m = LOG_LINE.search(line)
        if m:
            epoch, epochs, sample, samples, loss, rate = m.groups()
            left = (int(epochs) - int(epoch)) * int(samples) + int(samples) - int(sample)
            append(run, {"epoch": epoch, "epochs": epochs, "sample": sample, "samples": samples, "loss": loss,
                         "sec_per_sample": rate, "eta_min": round(left * float(rate) / 60)})


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path)
    parser.add_argument("--from-log", type=Path)
    parser.add_argument("--watch", type=float, default=0, help="N초마다 로그를 다시 읽어 갱신 (이전 실행용)")
    args = parser.parse_args()
    args.run.mkdir(parents=True, exist_ok=True)
    while True:
        if args.from_log:
            from_log(args.run, args.from_log)
        write_report(args.run)
        print(f"[report] {args.run / 'report.html'}", flush=True)
        if not args.watch:
            break
        rows = read(args.run)
        if rows and rows[-1]["epoch"] == rows[-1]["epochs"] and rows[-1]["sample"] == rows[-1]["samples"]:
            break
        time.sleep(args.watch)


if __name__ == "__main__":
    main()
