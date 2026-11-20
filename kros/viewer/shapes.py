"""절차적 3D 도형 (모델 파일 없이 GPU로 그린다). 상자는 원점 중심, 원기둥은 원점 중심 + 축이 Z."""
import math
from collections.abc import Sequence

from panda3d.core import Geom, GeomNode, GeomTriangles, GeomVertexData, GeomVertexFormat, GeomVertexWriter, NodePath

Vec = tuple[float, float, float]
Tri = Sequence[tuple[Vec, Vec]]  # 꼭짓점 3개의 (위치, 법선). 바깥에서 봤을 때 반시계가 앞면


def mesh(name: str, triangles: Sequence[Tri]) -> NodePath:
    vdata = GeomVertexData(name, GeomVertexFormat.getV3n3(), Geom.UHStatic)
    vdata.setNumRows(len(triangles) * 3)
    vertex, normal = GeomVertexWriter(vdata, "vertex"), GeomVertexWriter(vdata, "normal")
    for tri in triangles:
        for pos, n in tri:
            vertex.addData3(*pos)
            normal.addData3(*n)
    prim = GeomTriangles(Geom.UHStatic)
    prim.addConsecutiveVertices(0, len(triangles) * 3)
    prim.closePrimitive()
    geom = Geom(vdata)
    geom.addPrimitive(prim)
    node = GeomNode(name)
    node.addGeom(geom)
    return NodePath(node)


def box(sx: float, sy: float, sz: float) -> NodePath:
    half = (sx / 2, sy / 2, sz / 2)
    tris: list[Tri] = []
    for axis in range(3):
        for sign in (1, -1):
            u, v = (axis + 1) % 3, (axis + 2) % 3
            if sign < 0:
                u, v = v, u  # u x v가 항상 바깥 법선이 되도록

            def corner(su: int, sv: int) -> Vec:
                p = [0.0, 0.0, 0.0]
                p[axis], p[u], p[v] = sign * half[axis], su * half[u], sv * half[v]
                return (p[0], p[1], p[2])

            n = [0.0, 0.0, 0.0]
            n[axis] = float(sign)
            nv: Vec = (n[0], n[1], n[2])
            c = [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)]
            tris += [[(c[0], nv), (c[1], nv), (c[2], nv)], [(c[0], nv), (c[2], nv), (c[3], nv)]]
    return mesh("box", tris)


def cylinder(radius: float, height: float, segments: int = 32) -> NodePath:
    h = height / 2
    tris: list[Tri] = []
    for i in range(segments):
        a0, a1 = 2 * math.pi * i / segments, 2 * math.pi * (i + 1) / segments
        c0, s0, c1, s1 = math.cos(a0), math.sin(a0), math.cos(a1), math.sin(a1)
        b0, b1 = (radius * c0, radius * s0, -h), (radius * c1, radius * s1, -h)
        t0, t1 = (radius * c0, radius * s0, h), (radius * c1, radius * s1, h)
        n0, n1 = (c0, s0, 0.0), (c1, s1, 0.0)
        tris += [[(b0, n0), (b1, n1), (t1, n1)], [(b0, n0), (t1, n1), (t0, n0)]]
        up, down = (0.0, 0.0, 1.0), (0.0, 0.0, -1.0)
        tris += [[((0.0, 0.0, h), up), (t0, up), (t1, up)], [((0.0, 0.0, -h), down), (b1, down), (b0, down)]]
    return mesh("cylinder", tris)
