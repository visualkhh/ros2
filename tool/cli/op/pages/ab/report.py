"""A/B rosbag → HTML report (inline SVG, matplotlib 불필요).

  python3 -m ...  대신 `./run-cli ab rl_nav_controller report --label ab1`가 호출한다.
  - A(RL) bag과 B(inner) bag에서 /cmd_vel_nav · /cmd_vel · /odom · /imu · FollowPath/status를 읽는다.
  - 시간에 따른 v/w, 실제 궤적, 주행 지표를 HTML 한 파일로 만든다.
"""
from __future__ import annotations

import html
import math
import time
from pathlib import Path

WANTED = (
    '/cmd_vel_nav', '/cmd_vel', '/cmd_vel_smoothed', '/cmd_vel_base',
    '/odom', '/imu', '/controller_server/FollowPath/status',
)
CMD_TOPICS = ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_smoothed', '/cmd_vel_base')


def _twist(msg) -> tuple[float, float]:
    if hasattr(msg, 'twist'):   # TwistStamped
        msg = msg.twist
    return float(msg.linear.x), float(msg.angular.z)


def _load(path: Path) -> dict:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage = 'mcap'
    try:
        import yaml
        meta = yaml.safe_load((path / 'metadata.yaml').read_text())
        storage = meta['rosbag2_bagfile_information'].get('storage_identifier', storage)
    except Exception:
        pass
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id=storage), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    rows = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in WANTED or topic not in types:
            continue
        try:
            msg = deserialize_message(data, get_message(types[topic]))
        except Exception:
            continue
        rows.append((topic, int(t), msg))
    if not rows:
        raise RuntimeError(f'bag에 필요한 토픽이 없음: {path}')

    t0 = min(t for _, t, _ in rows)
    cmd_topic = next((x for x in CMD_TOPICS if any(tp == x for tp, _, _ in rows)), None)
    cmd, odom, imu, status = [], [], [], []
    for topic, t, msg in rows:
        sec = (t - t0) / 1e9
        if topic == cmd_topic:
            v, w = _twist(msg)
            cmd.append((sec, v, w))
        elif topic == '/odom':
            p = msg.pose.pose.position
            v = float(msg.twist.twist.linear.x)
            w = float(msg.twist.twist.angular.z)
            odom.append((sec, v, w, float(p.x), float(p.y)))
        elif topic == '/imu':
            imu.append((sec, float(msg.angular_velocity.z)))
        elif topic == '/controller_server/FollowPath/status':
            status.append((sec, [float(x) for x in msg.data]))

    duration = max((r[0] for r in cmd + odom + imu + status), default=0.0)
    dist = sum(math.hypot(odom[i][3] - odom[i - 1][3], odom[i][4] - odom[i - 1][4])
               for i in range(1, len(odom)))
    speeds = [abs(v) for _, v, _ in odom] or [abs(v) for _, v, _ in cmd]
    angs = [abs(w) for _, _, w in odom] or [abs(w) for _, _, w in cmd]
    active = sum(1 for _, v, w in cmd if abs(v) > 0.01 or abs(w) > 0.03)
    rl = sum(1 for _, d in status if d and d[0] >= 0.5)
    loaded = sum(1 for _, d in status if len(d) > 2 and d[2] >= 0.5)
    return {
        'path': path, 'duration': duration, 'distance': dist,
        'samples': len(odom), 'cmd_topic': cmd_topic or '-',
        'cmd': cmd, 'odom': odom, 'imu': imu, 'status': status,
        'avg_speed': sum(speeds) / len(speeds) if speeds else 0.0,
        'max_speed': max(speeds, default=0.0),
        'avg_w': sum(angs) / len(angs) if angs else 0.0,
        'max_w': max(angs, default=0.0),
        'active': (active / len(cmd) * 100.0) if cmd else None,
        'rl_pct': (rl / len(status) * 100.0) if status else None,
        'loaded_pct': (loaded / len(status) * 100.0) if status else None,
    }


def _esc(x) -> str:
    return html.escape(str(x))


def _svg(series: list[dict], *, width=760, height=250, title='', xlabel='t (s)',
         ylabel='', zero=False) -> str:
    xs = [x for s in series for x in s['xs']]
    ys = [y for s in series for y in s['ys']]
    if not xs or not ys:
        return f'<p class="muted">[{_esc(title)}] 데이터 없음</p>'
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if zero:
        ymin, ymax = min(ymin, 0.0), max(ymax, 0.0)
    if xmax - xmin < 1e-9:
        xmax = xmin + 1.0
    if ymax - ymin < 1e-9:
        ymin, ymax = ymin - 0.5, ymax + 0.5
    pad_y = (ymax - ymin) * 0.08
    ymin, ymax = ymin - pad_y, ymax + pad_y
    ml, mr, mt, mb = 58, 14, 28, 32
    iw, ih = width - ml - mr, height - mt - mb

    def X(x):
        return ml + (x - xmin) / (xmax - xmin) * iw

    def Y(y):
        return mt + (ymax - y) / (ymax - ymin) * ih

    out = [f'<svg viewBox="0 0 {width} {height}" class="chart" xmlns="http://www.w3.org/2000/svg">',
           f'<text x="{ml}" y="17" class="t">{_esc(title)}</text>']
    for i in range(5):   # 가로 그리드
        y = ymin + (ymax - ymin) * i / 4
        yy = Y(y)
        out.append(f'<line x1="{ml}" y1="{yy:.1f}" x2="{width-mr}" y2="{yy:.1f}" class="g"/>')
        out.append(f'<text x="{ml-6}" y="{yy+4:.1f}" class="yl">{y:.2f}</text>')
    for i in range(5):   # 세로 그리드
        x = xmin + (xmax - xmin) * i / 4
        xx = X(x)
        out.append(f'<line x1="{xx:.1f}" y1="{mt}" x2="{xx:.1f}" y2="{height-mb}" class="g"/>')
        out.append(f'<text x="{xx:.1f}" y="{height-mb+16}" class="xl">{x:.1f}</text>')
    for s in series:
        pts = ' '.join(f'{X(x):.1f},{Y(y):.1f}' for x, y in zip(s['xs'], s['ys']))
        dash = ' stroke-dasharray="5 4"' if s.get('dash') else ''
        out.append(f'<polyline points="{pts}" fill="none" stroke="{s["color"]}" '
                   f'stroke-width="{s.get("width", 2)}"{dash}/>')
    lx = ml + 4
    for s in series:
        out.append(f'<line x1="{lx}" y1="{height-8}" x2="{lx+18}" y2="{height-8}" '
                   f'stroke="{s["color"]}" stroke-width="3"/>')
        out.append(f'<text x="{lx+23}" y="{height-4}" class="lg">{_esc(s["label"])}</text>')
        lx += 34 + len(s['label']) * 8
    out.append(f'<text x="{width/2}" y="{height-2}" class="lbl" text-anchor="middle">{_esc(xlabel)}</text>')
    if ylabel:
        out.append(f'<text x="12" y="{mt+ih/2}" class="lbl" transform="rotate(-90 12 {mt+ih/2})" '
                   f'text-anchor="middle">{_esc(ylabel)}</text>')
    out.append('</svg>')
    return ''.join(out)


def _metric_row(name, a, b, fmt='{:.2f}'):
    def f(v):
        if v is None:
            return '-'
        return fmt.format(v)
    return f'<tr><th>{_esc(name)}</th><td>{f(a)}</td><td>{f(b)}</td></tr>'


def generate_report(a_path: Path, b_path: Path, label: str | None = None,
                    out: Path | None = None) -> Path:
    a, b = _load(Path(a_path)), _load(Path(b_path))
    label = label or time.strftime('%Y%m%d_%H%M%S')
    out = Path(out) if out else a_path.parent / f'report_{label}_{time.strftime("%H%M%S")}.html'

    def series_cmd(key):
        return [dict(label='A RL', color='#e5484d', xs=[x[0] for x in a['cmd']], ys=[x[key] for x in a['cmd']]),
                dict(label='B inner', color='#2f81f7', xs=[x[0] for x in b['cmd']], ys=[x[key] for x in b['cmd']])]

    def series_odom(key):
        idx = 1 if key == 'v' else 2
        return [dict(label='A RL', color='#e5484d', xs=[x[0] for x in a['odom']], ys=[x[idx] for x in a['odom']]),
                dict(label='B inner', color='#2f81f7', xs=[x[0] for x in b['odom']], ys=[x[idx] for x in b['odom']])]

    xy = [dict(label='A RL', color='#e5484d', xs=[x[3] for x in a['odom']], ys=[x[4] for x in a['odom']]),
          dict(label='B inner', color='#2f81f7', xs=[x[3] for x in b['odom']], ys=[x[4] for x in b['odom']])]

    rows = [
        _metric_row('주행 시간 (s)', a['duration'], b['duration']),
        _metric_row('이동 거리 (m)', a['distance'], b['distance']),
        _metric_row('평균 속도 (m/s)', a['avg_speed'], b['avg_speed']),
        _metric_row('최고 속도 (m/s)', a['max_speed'], b['max_speed']),
        _metric_row('평균 |ω| (rad/s)', a['avg_w'], b['avg_w']),
        _metric_row('최고 |ω| (rad/s)', a['max_w'], b['max_w']),
        _metric_row('명령 활동 (%)', a['active'], b['active'], '{:.0f}'),
        _metric_row('RL active (%)', a['rl_pct'], b['rl_pct'], '{:.0f}'),
        _metric_row('모델 loaded (%)', a['loaded_pct'], b['loaded_pct'], '{:.0f}'),
        f'<tr><th>cmd 토픽</th><td>{_esc(a["cmd_topic"])}</td><td>{_esc(b["cmd_topic"])}</td></tr>',
    ]
    body = f'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<title>RL vs inner A/B — {_esc(label)}</title>
<style>
body{{font-family:system-ui,-apple-system,"Noto Sans KR",sans-serif;margin:24px;color:#1f2328;background:#fff}}
h1{{font-size:22px;margin:0 0 4px}} h2{{font-size:16px;margin:24px 0 8px}}
.muted,.lbl,.yl,.xl,.lg{{fill:#57606a;font-size:11px}} .t{{font-size:13px;font-weight:600}}
table{{border-collapse:collapse;margin:8px 0 16px;min-width:520px}}
th,td{{border:1px solid #d0d7de;padding:6px 10px;text-align:right}} th{{text-align:left;background:#f6f8fa}}
td:first-child,th:first-child{{text-align:left}}
.chart{{width:100%;max-width:900px;background:#fff;border:1px solid #d0d7de;border-radius:6px;margin:6px 0}}
.g{{stroke:#eaeef2;stroke-width:1}} code{{background:#f6f8fa;padding:2px 5px;border-radius:4px}}
</style></head><body>
<h1>RL vs inner controller A/B</h1>
<p class="muted">{_esc(label)} · {time.strftime('%Y-%m-%d %H:%M:%S')}</p>
<p>A: <code>{_esc(a_path)}</code><br>B: <code>{_esc(b_path)}</code></p>
<h2>지표</h2>
<table><thead><tr><th>항목</th><th>A (RL)</th><th>B (inner)</th></tr></thead><tbody>
{''.join(rows)}
</tbody></table>
<p class="muted">RL active는 <code>/controller_server/FollowPath/status</code>의 첫 값이 0.5 이상인 비율입니다.
B(inner)는 0이 정상입니다.</p>
<h2>명령 속도 v</h2>{_svg(series_cmd(1), title='cmd v', ylabel='m/s', zero=True)}
<h2>명령 각속도 ω</h2>{_svg(series_cmd(2), title='cmd ω', ylabel='rad/s', zero=True)}
<h2>실제 속도 v (odom)</h2>{_svg(series_odom('v'), title='odom v', ylabel='m/s', zero=True)}
<h2>실제 각속도 ω (odom)</h2>{_svg(series_odom('w'), title='odom ω', ylabel='rad/s', zero=True)}
<h2>궤적 (odom)</h2>{_svg(xy, title='XY trajectory', xlabel='x (m)', ylabel='y (m)')}
</body></html>'''
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(body, encoding='utf-8')
    return out
