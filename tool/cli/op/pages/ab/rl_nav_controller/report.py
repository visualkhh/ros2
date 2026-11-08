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
    '/odom', '/imu', '/amcl_pose', '/controller_server/FollowPath/status',
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
    cmd, odom, imu, status, amcl = [], [], [], [], []
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
        elif topic == '/amcl_pose':
            p = msg.pose.pose.position
            amcl.append((sec, float(p.x), float(p.y)))
        elif topic == '/imu':
            imu.append((sec, float(msg.angular_velocity.z)))
        elif topic == '/controller_server/FollowPath/status':
            status.append((sec, [float(x) for x in msg.data]))

    duration = max((r[0] for r in cmd + odom + imu + status), default=0.0)
    dist = sum(math.hypot(odom[i][3] - odom[i - 1][3], odom[i][4] - odom[i - 1][4])
               for i in range(1, len(odom)))
    speeds = [abs(o[1]) for o in odom] or [abs(c[1]) for c in cmd]
    angs = [abs(o[2]) for o in odom] or [abs(c[2]) for c in cmd]
    active = sum(1 for _, v, w in cmd if abs(v) > 0.01 or abs(w) > 0.03)
    rl = sum(1 for _, d in status if d and d[0] >= 0.5)
    loaded = sum(1 for _, d in status if len(d) > 2 and d[2] >= 0.5)
    return {
        'path': path, 'duration': duration, 'run_time': max(0.0, duration - 2.0), 'distance': dist,
        'samples': len(odom), 'cmd_topic': cmd_topic or '-',
        'cmd': cmd, 'odom': odom, 'imu': imu, 'amcl': amcl, 'status': status,
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


def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _path_stats(odom, amcl, start, route):
    map_pts = [(t, x, y) for t, x, y in amcl] or \
        [(t, x, y) for t, _, _, x, y in odom]
    pts = [(x, y) for _, x, y in map_pts]
    if not pts:
        return dict(lat_rms=None, lat_max=None, goal_err=None, efficiency=None,
                    drive_time=0.0, stop_frac=None, alpha=None)
    s = start or pts[0]
    goals = list(route) or [pts[-1]]
    poly = [s] + goals
    errs = [min(_seg_dist(px, py, *poly[i], *poly[i + 1])
                for i in range(len(poly) - 1)) for px, py in pts]
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    straight = sum(math.hypot(poly[i + 1][0] - poly[i][0], poly[i + 1][1] - poly[i][1])
                   for i in range(len(poly) - 1))
    dist = sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
               for i in range(1, len(pts)))
    goal = goals[-1]
    t_end = map_pts[-1][0]
    near = [(x, y) for t, x, y in map_pts if t_end - t <= 1.5] or [pts[-1]]
    goal_err = min(math.hypot(x - goal[0], y - goal[1]) for x, y in near)
    moving = [o for o in odom if abs(o[1]) > 0.01]
    drive_time = (moving[-1][0] - moving[0][0]) if len(moving) > 1 else (
        odom[-1][0] - odom[0][0] if len(odom) > 1 else 0.0)
    stop_frac = sum(1 for o in odom if abs(o[1]) < 0.01) / len(odom) * 100.0
    alphas = []
    for i in range(1, len(odom)):
        dt = odom[i][0] - odom[i - 1][0]
        if 1e-3 < dt < 0.5:
            alphas.append(abs(odom[i][2] - odom[i - 1][2]) / dt)
    alpha = sum(alphas) / len(alphas) if alphas else None
    return dict(lat_rms=rms, lat_max=max(errs), goal_err=goal_err,
                efficiency=(straight / dist * 100.0) if dist > 1e-6 else None,
                drive_time=drive_time, stop_frac=stop_frac, alpha=alpha)


def _pct(a, b, lower: bool) -> float | None:
    if a is None or b is None:
        return None
    if lower:
        if a <= b:
            return (b - a) / max(abs(b), 1e-6) * 100.0
        return -(a - b) / max(abs(a), 1e-6) * 100.0
    if a >= b:
        return (a - b) / max(abs(b), 1e-6) * 100.0
    return -(b - a) / max(abs(a), 1e-6) * 100.0


def _row(name, a, b, *, lower=None, fmt='{:.2f}', unit='', word=''):
    def f(v):
        return '-' if v is None else fmt.format(v) + (f' {unit}' if unit else '')
    diff, cls = '-', ''
    if lower is not None:
        p = _pct(a, b, lower)
        if p is None:
            pass
        elif abs(p) <= 3.0:
            diff = '비슷'
        elif p > 0:
            diff, cls = f'A 우세 ({p:.0f}% {word})', 'good'
        else:
            diff, cls = f'B 우세 ({-p:.0f}% {word})', 'bad'
    return f'<tr><th>{_esc(name)}</th><td>{f(a)}</td><td>{f(b)}</td><td class="{cls}">{diff}</td></tr>'


def generate_report(a_path: Path, b_path: Path, label: str | None = None,
                    out: Path | None = None, start=None, route=None) -> Path:
    a, b = _load(Path(a_path)), _load(Path(b_path))
    for x in (a, b):
        x.update(_path_stats(x['odom'], x['amcl'], start, route))
    label = label or time.strftime('%Y%m%d_%H%M%S')
    out = Path(out) if out else a_path.parent / f'report_{label}_{time.strftime("%H%M%S")}.html'

    def series_cmd(key):
        return [dict(label='A RL', color='#e5484d', xs=[x[0] for x in a['cmd']], ys=[x[key] for x in a['cmd']]),
                dict(label='B inner', color='#2f81f7', xs=[x[0] for x in b['cmd']], ys=[x[key] for x in b['cmd']])]

    def series_odom(key):
        idx = 1 if key == 'v' else 2
        return [dict(label='A RL', color='#e5484d', xs=[x[0] for x in a['odom']], ys=[x[idx] for x in a['odom']]),
                dict(label='B inner', color='#2f81f7', xs=[x[0] for x in b['odom']], ys=[x[idx] for x in b['odom']])]

    xy = [dict(label='A RL', color='#e5484d',
               xs=[x[1] for x in a['amcl']] or [x[3] for x in a['odom']],
               ys=[x[2] for x in a['amcl']] or [x[4] for x in a['odom']]),
          dict(label='B inner', color='#2f81f7',
               xs=[x[1] for x in b['amcl']] or [x[3] for x in b['odom']],
               ys=[x[2] for x in b['amcl']] or [x[4] for x in b['odom']])]
    xy_odom = [dict(label='A RL', color='#e5484d',
                    xs=[x[3] for x in a['odom']], ys=[x[4] for x in a['odom']]),
               dict(label='B inner', color='#2f81f7',
                    xs=[x[3] for x in b['odom']], ys=[x[4] for x in b['odom']])]

    ok_a = a['goal_err'] is not None and a['goal_err'] <= 0.30 and a['distance'] > 0.5
    ok_b = b['goal_err'] is not None and b['goal_err'] <= 0.30 and b['distance'] > 0.5
    rows = [
        _row('주행 시간 (s)', a['run_time'], b['run_time'], lower=True, word='빠름'),
        _row('주행 거리 (m)', a['distance'], b['distance']),
        _row('평균 속도 (m/s)', a['avg_speed'], b['avg_speed']),
        _row('최고 속도 (m/s)', a['max_speed'], b['max_speed']),
        _row('경로 오차 RMS (m)', a['lat_rms'], b['lat_rms'], lower=True, unit='m', word='작음'),
        _row('경로 오차 최대 (m)', a['lat_max'], b['lat_max'], lower=True, unit='m', word='작음'),
        _row('도착 오차 (m)', a['goal_err'], b['goal_err'], lower=True, unit='m', word='작음'),
        _row('평균 |ω| (rad/s)', a['avg_w'], b['avg_w'], lower=True, word='작음'),
        _row('평균 각가속도 (rad/s²)', a['alpha'], b['alpha'], lower=True, word='부드러움'),
        _row('정지 비율 (%)', a['stop_frac'], b['stop_frac'], lower=True, fmt='{:.0f}', word='작음'),
        _row('RL active (%)', a['rl_pct'], b['rl_pct'], fmt='{:.0f}'),
        _row('모델 loaded (%)', a['loaded_pct'], b['loaded_pct'], fmt='{:.0f}'),
        f'<tr><th>완주</th><td class="{"good" if ok_a else "bad"}">{"예" if ok_a else "아니오"}</td>'
        f'<td class="{"good" if ok_b else "bad"}">{"예" if ok_b else "아니오"}</td>'
        f'<td class="{"good" if ok_a and not ok_b else "bad" if ok_b and not ok_a else ""}">'
        f'{"A만 완주" if ok_a and not ok_b else "B만 완주" if ok_b and not ok_a else "-"}</td></tr>',
        f'<tr><th>cmd 토픽</th><td>{_esc(a["cmd_topic"])}</td><td>{_esc(b["cmd_topic"])}</td><td>-</td></tr>',
    ]

    # 종합 판정: 핵심 5개 지표 가중치
    scored = [
        ('주행 시간', a['run_time'], b['run_time'], True, 25),
        ('경로 오차', a['lat_rms'], b['lat_rms'], True, 30),
        ('도착 오차', a['goal_err'], b['goal_err'], True, 20),
        ('부드러움', a['alpha'], b['alpha'], True, 15),
        ('정지 비율', a['stop_frac'], b['stop_frac'], True, 10),
    ]
    aw = bw = 0
    a_wins, b_wins = [], []
    for name, av, bv, lower, w in scored:
        p = _pct(av, bv, lower)
        if p is None:
            continue
        if p > 3.0:
            aw += w
            a_wins.append(name)
        elif p < -3.0:
            bw += w
            b_wins.append(name)
    total = aw + bw
    if total == 0:
        verdict, vcls = '비슷', 'neutral'
    elif aw > bw * 1.2:
        verdict, vcls = 'A (RL) 우세', 'good'
    elif bw > aw * 1.2:
        verdict, vcls = 'B (inner) 우세', 'bad'
    else:
        verdict, vcls = '비슷', 'neutral'

    highlights = []
    for name, av, bv, lower, word in (
            ('주행 시간', a['run_time'], b['run_time'], True, '빠름'),
            ('경로 이탈', a['lat_rms'], b['lat_rms'], True, '작음'),
            ('도착 오차', a['goal_err'], b['goal_err'], True, '작음'),
            ('각가속도', a['alpha'], b['alpha'], True, '부드러움')):
        p = _pct(av, bv, lower)
        if p is None or abs(p) <= 3.0:
            continue
        who = 'A(RL)' if p > 0 else 'B(inner)'
        highlights.append(f'{who}가 {name} {abs(p):.0f}% {word}')
    summary = ' · '.join(highlights[:4]) if highlights else '주요 지표 차이 3% 이내 — 비슷'
    wins_html = (f'<p class="muted">A 우세: {", ".join(a_wins) or "없음"}'
                 f' · B 우세: {", ".join(b_wins) or "없음"}</p>')

    score_bar = ''
    if total:
        ap = aw / total * 100.0
        score_bar = (f'<div class="score"><div class="scoreA" style="width:{ap:.0f}%">'
                     f'A {aw}</div><div class="scoreB" style="width:{100-ap:.0f}%">B {bw}</div></div>'
                     f'<p class="muted">가중치: 시간 25 · 경로오차 30 · 도착오차 20 · 부드러움 15 · 정지 10</p>')

    body = f'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<title>RL vs inner A/B — {_esc(label)}</title>
<style>
body{{font-family:system-ui,-apple-system,"Noto Sans KR",sans-serif;margin:24px;color:#1f2328;background:#fff}}
h1{{font-size:22px;margin:0 0 4px}} h2{{font-size:16px;margin:24px 0 8px}}
.muted,.lbl,.yl,.xl,.lg{{fill:#57606a;font-size:11px}} .t{{font-size:13px;font-weight:600}}
table{{border-collapse:collapse;margin:8px 0 16px;min-width:720px}}
th,td{{border:1px solid #d0d7de;padding:6px 10px;text-align:right}} th{{text-align:left;background:#f6f8fa}}
td:first-child,th:first-child{{text-align:left}} td:nth-child(4){{text-align:left;font-weight:600}}
.good{{color:#1a7f37}} .bad{{color:#cf222e}} .neutral{{color:#57606a}}
.chart{{width:100%;max-width:900px;background:#fff;border:1px solid #d0d7de;border-radius:6px;margin:6px 0}}
.g{{stroke:#eaeef2;stroke-width:1}} code{{background:#f6f8fa;padding:2px 5px;border-radius:4px}}
.hero{{border:1px solid #d0d7de;border-radius:8px;padding:14px 16px;margin:14px 0;background:#f6f8fa}}
.verdict{{font-size:20px;font-weight:700;margin-bottom:4px}} .verdict.good{{color:#1a7f37}} .verdict.bad{{color:#cf222e}} .verdict.neutral{{color:#57606a}}
.score{{display:flex;height:22px;border-radius:5px;overflow:hidden;margin:10px 0 2px;font-size:12px;color:#fff;font-weight:600}}
.scoreA{{background:#e5484d;display:flex;align-items:center;justify-content:center;min-width:2em}}
.scoreB{{background:#2f81f7;display:flex;align-items:center;justify-content:center;min-width:2em}}
</style></head><body>
<h1>RL vs inner controller A/B</h1>
<p class="muted">{_esc(label)} · {time.strftime('%Y-%m-%d %H:%M:%S')}</p>
<p>A: <code>{_esc(a_path)}</code><br>B: <code>{_esc(b_path)}</code></p>
<div class="hero">
  <div class="verdict {vcls}">종합: {verdict}</div>
  <div>{_esc(summary)}</div>
  {wins_html}
  {score_bar}
</div>
<h2>지표 비교</h2>
<table><thead><tr><th>항목</th><th>A (RL)</th><th>B (inner)</th><th>차이 / 판정</th></tr></thead><tbody>
{''.join(rows)}
</tbody></table>
<p class="muted">주행 시간은 bag 시작의 2초 대기와 정지 구간을 제외한 실제 이동 구간입니다.
RL active는 <code>/controller_server/FollowPath/status</code>의 첫 값이 0.5 이상인 비율입니다.
B(inner)는 0이 정상입니다.</p>
<h2>odom 위치 궤적 (raw)</h2>{_svg(xy_odom, title='odom 위치 궤적', xlabel='x (m)', ylabel='y (m)')}
<h2>map 위치 궤적 (amcl_pose)</h2>{_svg(xy, title='map 위치 궤적', xlabel='x (m)', ylabel='y (m)')}
<p class="muted">경로·도착 오차는 map(amcl_pose) 기준입니다. amcl_pose가 없으면 /odom으로 대체합니다.
odom 궤적은 원본 odom 프레임이라 map 궤적과 원점이 다를 수 있습니다.</p>
<h2>명령 속도 v</h2>{_svg(series_cmd(1), title='cmd v', ylabel='m/s', zero=True)}
<h2>명령 각속도 ω</h2>{_svg(series_cmd(2), title='cmd ω', ylabel='rad/s', zero=True)}
<h2>실제 속도 v (odom)</h2>{_svg(series_odom('v'), title='odom v', ylabel='m/s', zero=True)}
<h2>실제 각속도 ω (odom)</h2>{_svg(series_odom('w'), title='odom ω', ylabel='rad/s', zero=True)}
</body></html>'''
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(body, encoding='utf-8')
    return out
