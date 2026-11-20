import math
import re
import struct
from pathlib import Path

from panda3d.core import NodePath

from .shapes import Tri, Vec, mesh


def _normal(a: Vec, b: Vec, c: Vec) -> Vec:
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return (nx / length, ny / length, nz / length)


def _vertices(data: bytes) -> list[tuple[float, ...]]:
    """STL(binary 또는 ascii)의 삼각형들을 (x1,y1,z1, x2,y2,z2, x3,y3,z3)로 돌려준다."""
    if len(data) >= 84 and len(data) == 84 + 50 * struct.unpack_from("<I", data, 80)[0]:
        return [r[3:12] for r in struct.iter_unpack("<12fH", data[84:])]  # r[0:3]은 파일의 법선(안 씀)
    numbers = [float(x) for x in re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data) for x in x]
    return [tuple(numbers[i : i + 9]) for i in range(0, len(numbers), 9)]


def _triangles(path: Path, scale: tuple[float, float, float]) -> list[Tri]:
    sx, sy, sz = scale
    triangles: list[Tri] = []
    for t in _vertices(path.read_bytes()):
        a, b, c = (t[0] * sx, t[1] * sy, t[2] * sz), (t[3] * sx, t[4] * sy, t[5] * sz), (t[6] * sx, t[7] * sy, t[8] * sz)
        n = _normal(a, b, c)
        triangles.append([(a, n), (b, n), (c, n)])
    return triangles


def load_stl(path: Path, scale: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> NodePath:
    """STL 파일을 Panda3D 메시로 읽는다. 법선은 삼각형 면에서 계산(면 단위 음영)."""
    return mesh(path.stem, _triangles(path, scale))
