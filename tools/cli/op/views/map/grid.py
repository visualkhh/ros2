"""MapGrid — nav_msgs/OccupancyGrid를 터미널 그리기용으로 줄인 것 (벽 우선 피라미드)."""
from __future__ import annotations

import math

UNKNOWN, FREE, OCC, OUT = 0, 1, 2, 3   # 칸 종류 — 큰 값이 이긴다 (확대 안 한 먼 화면에서도 벽이 안 사라지게). OUT = 지도 밖


class MapGrid:
    """width × height 칸, 해상도 res [m], 왼쪽 아래 모서리 (ox, oy). level(k) = 2^k × 2^k 칸을 하나로 합친 지도."""

    def __init__(self, width: int, height: int, res: float, ox: float, oy: float, data,
                 yaw: float = 0.0):
        self.w, self.h, self.res, self.ox, self.oy = width, height, res, ox, oy
        self.yaw = float(yaw)   # map 기준 격자 회전 [rad] — map.yaml origin 3번째 값
        raw = bytearray(width * height)
        for i, v in enumerate(data):
            raw[i] = UNKNOWN if v < 0 else (OCC if v >= 65 else FREE)
        self._levels = [(width, height, bytes(raw))]

    @classmethod
    def from_msg(cls, m) -> 'MapGrid':
        i = m.info
        q = i.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return cls(int(i.width), int(i.height), float(i.resolution), float(i.origin.position.x),
                   float(i.origin.position.y), m.data, yaw)

    @classmethod
    def from_yaml(cls, path) -> 'MapGrid':
        """저장된 지도 파일(map_server 형식: yaml + pgm) → MapGrid. 읽을 수 없으면 RuntimeError."""
        from pathlib import Path
        path = Path(path)
        try:
            cfg = {}
            for line in path.read_text().splitlines():
                if ':' in line and not line.lstrip().startswith('#'):
                    k, v = line.split(':', 1)
                    cfg[k.strip()] = v.strip()
            raw = (path.parent / cfg['image']).read_bytes()
            toks, pos = [], 0
            while len(toks) < 4:   # P5 · 폭 높이 · 최대값 (# 주석 줄 건너뜀)
                while raw[pos:pos + 1].isspace():
                    pos += 1
                if raw[pos:pos + 1] == b'#':
                    pos = raw.index(b'\n', pos) + 1
                    continue
                end = pos
                while not raw[end:end + 1].isspace():
                    end += 1
                toks.append(raw[pos:end])
                pos = end
            if toks[0] != b'P5':
                raise ValueError(f'P5 pgm만 지원: {toks[0]!r}')
            w, h = int(toks[1]), int(toks[2])
            pix = raw[pos + 1:pos + 1 + w * h]
            occ_t, free_t = float(cfg.get('occupied_thresh', 0.65)), float(cfg.get('free_thresh', 0.25))
            neg = cfg.get('negate', '0') in ('1', 'true')
            origin = [float(t) for t in cfg['origin'].strip('[]').split(',')]
            ox, oy = origin[:2]
            yaw = origin[2] if len(origin) > 2 else 0.0   # map.yaml은 radian
            data = []
            for y in range(h - 1, -1, -1):   # pgm은 위→아래, 지도는 아래→위
                for x in range(w):
                    v = pix[y * w + x] / 255.0
                    p = v if neg else 1.0 - v
                    data.append(100 if p > occ_t else (0 if p < free_t else -1))
            return cls(w, h, float(cfg['resolution']), ox, oy, data, yaw)
        except (OSError, KeyError, ValueError, IndexError) as exc:
            raise RuntimeError(f'지도 파일을 못 읽음: {path} ({exc})') from None

    def level(self, k: int) -> tuple[int, int, bytes]:
        while len(self._levels) <= k:
            w, h, raw = self._levels[-1]
            w2, h2 = (w + 1) // 2, (h + 1) // 2
            out = bytearray()
            for y in range(h2):
                r0 = raw[2 * y * w:(2 * y + 1) * w]
                r1 = raw[(2 * y + 1) * w:(2 * y + 2) * w] or r0
                m = bytes(map(max, r0, r1))
                out += bytes(map(max, m[0::2], m[1::2] + b'\0' * (w2 - len(m[1::2]))))
            self._levels.append((w2, h2, bytes(out)))
        return self._levels[k]

    def center(self) -> tuple[float, float]:
        """격자 중심의 map 좌표 — origin 회전까지 적용."""
        dx, dy = self.w * self.res / 2.0, self.h * self.res / 2.0
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.ox + c * dx - s * dy, self.oy + s * dx + c * dy

    def size_m(self) -> tuple[float, float]:
        """map 축 기준 회전된 격자의 외곽 크기 (AABB) [m]."""
        w, h = self.w * self.res, self.h * self.res
        c, s = abs(math.cos(self.yaw)), abs(math.sin(self.yaw))
        return w * c + h * s, w * s + h * c

    def code(self, k: int, wx: float, wy: float) -> int:
        """세계 좌표 (wx, wy)가 level(k) 지도에서 어떤 칸인지 (k<0이면 level 0)."""
        lv = max(k, 0)
        w, h, raw = self.level(lv)
        size = self.res * (1 << lv)
        dx, dy = wx - self.ox, wy - self.oy
        if self.yaw:   # world → 격자 로컬 (역회전)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            dx, dy = c * dx + s * dy, -s * dx + c * dy
        cx, cy = math.floor(dx / size), math.floor(dy / size)
        return raw[cy * w + cx] if 0 <= cx < w and 0 <= cy < h else OUT

class CostGrid:
    """nav_msgs/OccupancyGrid 비용 지도 — 0 자유/모름, 1 팽창, 2 내접/치명.

    /global_costmap/costmap 값은 0~100 (99=내접, 100=치명), -1=모름.
    지도 화면에서 코스트맵을 겹쳐 볼 때만 쓴다.
    """

    def __init__(self, width: int, height: int, res: float, ox: float, oy: float, data,
                 frame_id: str = 'map'):
        self.w, self.h, self.res, self.ox, self.oy = width, height, res, ox, oy
        self.data = data
        self.frame_id = frame_id
        self.parent: tuple[float, float, float] | None = None   # map 기준 (x, y, yaw[rad])

    @classmethod
    def from_msg(cls, m) -> 'CostGrid':
        i = m.info
        frame = getattr(getattr(m, 'header', None), 'frame_id', '') or 'map'
        return cls(int(i.width), int(i.height), float(i.resolution), float(i.origin.position.x),
                   float(i.origin.position.y), m.data, frame)

    def set_parent(self, x: float, y: float, yaw: float) -> None:
        """이 격자의 프레임 → map 변환 (map 기준 x, y, yaw). local costmap은 odom이라 필요하다."""
        self.parent = (float(x), float(y), float(yaw))

    def code(self, wx: float, wy: float) -> int:
        if self.parent is not None:   # map → 이 프레임 (역변환)
            px, py, yaw = self.parent
            dx, dy = wx - px, wy - py
            wx = math.cos(yaw) * dx + math.sin(yaw) * dy
            wy = -math.sin(yaw) * dx + math.cos(yaw) * dy
        c = math.floor((wx - self.ox) / self.res)
        r = math.floor((wy - self.oy) / self.res)
        if not (0 <= c < self.w and 0 <= r < self.h):
            return 0
        v = self.data[r * self.w + c]
        if v < 0 or v == 0:
            return 0
        return 2 if v >= 99 else 1
