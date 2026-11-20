"""nav2_map_server의 map_io 흉내: 격자 지도를 PGM(그림) + YAML(메타데이터) 두 파일로 저장하고 읽는다. 표준 라이브러리만 쓴다.

YAML (ROS map_server 형식):
    image: map.pgm            # yaml 파일 기준 상대 경로
    mode: trinary
    resolution: 0.05          # 칸 크기 m
    origin: [x, y, yaw]       # 그림 왼쪽 아래 칸의 모서리가 map 좌표계에서 놓인 곳
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196

PGM(P5, 8비트 회색): 그림의 맨 윗줄이 지도의 가장 큰 y. trinary로 저장하면 벽 0(검정), 빈 곳 254(흰색), 모름 205(회색).
읽을 때는 점유 정도 occ = (255 - 픽셀) / 255 (negate면 반대)로 occupied_thresh보다 크면 벽, free_thresh보다 작으면 빈 곳, 아니면 모름.
free_thresh 기본값이 ROS1 map_saver처럼 0.196인 이유: 모름(205)의 occ가 50/255 = 0.19607이라 이보다 작아야 저장한 모름이 다시 모름으로 읽힌다.
"""
import math
from pathlib import Path

from kros.msgs.geometry_msgs import Point, Pose, Quaternion
from kros.msgs.nav_msgs import FREE, OCCUPIED, UNKNOWN, MapMetaData, OccupancyGrid
from kros.msgs.std_msgs import Header

FREE_THRESH, OCCUPIED_THRESH = 0.196, 0.65
WALL_PIXEL, FREE_PIXEL, UNKNOWN_PIXEL = 0, 254, 205


class MapIoError(Exception):
    """지도 파일이 없거나 형식이 틀렸다."""


def save_map(grid: OccupancyGrid, path: Path, free_thresh: float = FREE_THRESH, occupied_thresh: float = OCCUPIED_THRESH) -> Path:
    """path(확장자 없이)에 .pgm과 .yaml을 쓴다. 칸 값(0~100)은 occupied_thresh*100 이상이면 벽, free_thresh*100 이하면 빈 곳, 그 사이와 -1은 모름. yaml 경로를 돌려준다."""
    info = grid.info
    if info.width <= 0 or info.height <= 0 or len(grid.data) != info.width * info.height:
        raise MapIoError(f"지도 크기({info.width}x{info.height})와 칸 수({len(grid.data)})가 맞지 않는다")
    path.parent.mkdir(parents=True, exist_ok=True)

    def pixel(value: int) -> int:
        if value == UNKNOWN:
            return UNKNOWN_PIXEL
        if value >= occupied_thresh * 100:
            return WALL_PIXEL
        if value <= free_thresh * 100:
            return FREE_PIXEL
        return UNKNOWN_PIXEL

    rows = [bytes(pixel(v) for v in grid.data[y * info.width : (y + 1) * info.width]) for y in reversed(range(info.height))]  # 그림은 위(큰 y)부터
    image = path.with_suffix(".pgm")
    image.write_bytes(f"P5\n# CREATOR: kros map_saver {info.resolution:.3f} m/pix\n{info.width} {info.height}\n255\n".encode() + b"".join(rows))
    o, q = info.origin.position, info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    yaml = path.with_suffix(".yaml")
    yaml.write_text(
        f"image: {image.name}\nmode: trinary\nresolution: {info.resolution}\norigin: [{o.x}, {o.y}, {yaw}]\n"
        f"negate: 0\noccupied_thresh: {occupied_thresh}\nfree_thresh: {free_thresh}\n"
    )
    return yaml


def _parse_yaml(text: str) -> dict[str, str | list[str]]:
    """map_server yaml처럼 한 줄에 "키: 값" 또는 "키: [a, b, c]"만 있는 평평한 yaml을 읽는다 (PyYAML 없이)."""
    values: dict[str, str | list[str]] = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        key, sep, value = line.partition(":")
        if not sep:
            raise MapIoError(f"yaml 줄을 읽을 수 없다: {line!r}")
        value = value.strip()
        values[key.strip()] = [item.strip() for item in value[1:-1].split(",")] if value.startswith("[") and value.endswith("]") else value.strip("'\"")
    return values


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    """P5(8비트) PGM: (너비, 높이, 위 줄부터의 픽셀)."""
    data = path.read_bytes()
    tokens: list[bytes] = []
    i = 0
    while len(tokens) < 4:  # P5, 너비, 높이, 최댓값 (사이에 # 주석이 올 수 있다)
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            i = data.index(b"\n", i) + 1
            continue
        start = i
        while i < len(data) and not data[i : i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    if tokens[0] != b"P5" or int(tokens[3]) != 255:
        raise MapIoError(f"8비트 P5 PGM만 읽는다: {path}")
    width, height = int(tokens[1]), int(tokens[2])
    pixels = data[i + 1 : i + 1 + width * height]  # 최댓값 뒤의 공백 한 칸 다음부터
    if len(pixels) != width * height:
        raise MapIoError(f"PGM 픽셀 수가 모자란다: {path}")
    return width, height, pixels


def load_map(yaml_path: Path, frame_id: str = "map", stamp: float = 0.0) -> OccupancyGrid:
    """yaml(과 그 안의 image)을 읽어 OccupancyGrid로. 파일이 없으면 FileNotFoundError, 형식이 틀리면 MapIoError."""
    meta = _parse_yaml(yaml_path.read_text())
    try:
        image, resolution, origin = meta["image"], float(str(meta["resolution"])), [float(v) for v in meta["origin"]]
        negate = int(str(meta.get("negate", "0")))
        free_thresh = float(str(meta.get("free_thresh", FREE_THRESH)))
        occupied_thresh = float(str(meta.get("occupied_thresh", OCCUPIED_THRESH)))
        mode = str(meta.get("mode", "trinary"))
    except (KeyError, ValueError) as error:
        raise MapIoError(f"지도 yaml의 값이 틀렸다: {yaml_path}: {error}") from error
    if not isinstance(image, str) or len(origin) != 3 or resolution <= 0:
        raise MapIoError(f"지도 yaml의 image/origin/resolution이 틀렸다: {yaml_path}")
    if mode != "trinary":
        raise MapIoError(f"mode는 trinary만 지원한다: {mode}")  # shortcut: scale/raw 모드는 안 읽는다, 필요해지면 추가
    width, height, pixels = _read_pgm(yaml_path.parent / image)

    def cell(p: int) -> int:
        occ = p / 255 if negate else (255 - p) / 255
        return OCCUPIED if occ > occupied_thresh else FREE if occ < free_thresh else UNKNOWN

    data = tuple(cell(p) for y in reversed(range(height)) for p in pixels[y * width : (y + 1) * width])  # 지도는 아래(작은 y)부터
    x, y, yaw = origin
    pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    return OccupancyGrid(Header(stamp, frame_id), MapMetaData(stamp, resolution, width, height, pose), data)
