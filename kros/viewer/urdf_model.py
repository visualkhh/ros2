import math
import xml.etree.ElementTree as ET
from pathlib import Path

from panda3d.core import LMatrix4f, LQuaternionf, NodePath

from .stl import load_stl


def _floats(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(float(v) for v in text.split()) if text else default


def _origin_matrix(origin: ET.Element | None) -> LMatrix4f:
    """URDF <origin xyz rpy> -> Panda 행렬. R = Rz(yaw) Ry(pitch) Rx(roll)."""
    x, y, z = _floats(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
    r, p, w = _floats(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
    cr, sr, cp, sp, cw, sw = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(w), math.sin(w)
    m = [  # 열 벡터 규약의 회전행렬
        [cw * cp, cw * sp * sr - sw * cr, cw * sp * cr + sw * sr],
        [sw * cp, sw * sp * sr + cw * cr, sw * sp * cr - cw * sr],
        [-sp, cp * sr, cp * cr],
    ]
    # Panda는 행 벡터 규약이라 전치해서 넣는다 (이동은 마지막 행)
    return LMatrix4f(
        m[0][0], m[1][0], m[2][0], 0, m[0][1], m[1][1], m[2][1], 0, m[0][2], m[1][2], m[2][2], 0, x, y, z, 1
    )


def _link_name(joint: ET.Element, tag: str) -> str:
    element = joint.find(tag)
    name = element.get("link") if element is not None else None
    if name is None:
        raise ValueError(f"URDF joint {joint.get('name')}에 <{tag} link=...>가 없다")
    return name


class UrdfModel:
    """URDF의 link/joint 트리와 visual 메시를 Panda3D 노드로 만든다. continuous 조인트는 set_joint_angle로 돌린다.

    지원: mesh(STL) visual, material 색, fixed/continuous 조인트. (box/cylinder visual, mimic 등은 미지원)
    """

    def __init__(self, urdf_text: str, package_root: Path) -> None:
        """package_root: `package://<패키지>/` 뒤의 경로가 놓인 폴더."""
        xml = ET.fromstring(urdf_text.replace("${namespace}", ""))  # xacro 인자는 빈 값으로만 처리
        colors = {m.get("name"): _floats(m.find("color").get("rgba"), (1, 1, 1, 1)) for m in xml.findall("material")}  # type: ignore[union-attr]
        self.links: dict[str, NodePath] = {}
        for link in xml.findall("link"):
            node = NodePath(link.get("name") or "")
            self.links[node.getName()] = node
            for visual in link.findall("visual"):
                geometry = visual.find("geometry")
                mesh = geometry.find("mesh") if geometry is not None else None
                if mesh is None:
                    continue  # mesh가 아닌 visual은 건너뛴다 (box/cylinder 미지원)
                relative = mesh.get("filename", "").split("://", 1)[1].split("/", 1)[1]  # package://pkg/<relative>
                path = package_root / relative
                if not path.exists():
                    continue  # 메시 파일이 없으면 이 visual은 건너뛴다 (원격 viz 등)
                scale = _floats(mesh.get("scale"), (1.0, 1.0, 1.0))
                shape = load_stl(path, (scale[0], scale[1], scale[2]))
                material = visual.find("material")
                if material is not None and material.get("name") in colors:
                    shape.setColor(*colors[material.get("name")])
                shape.setMat(_origin_matrix(visual.find("origin")))
                shape.reparentTo(node)
        self._joints: dict[str, tuple[NodePath, tuple[float, ...]]] = {}
        children = set()
        for joint in xml.findall("joint"):
            parent, child = self.links[_link_name(joint, "parent")], self.links[_link_name(joint, "child")]
            frame = parent.attachNewNode(f"{joint.get('name')}_frame")
            frame.setMat(_origin_matrix(joint.find("origin")))
            child.reparentTo(frame)
            children.add(child.getName())
            if joint.get("type") == "continuous":
                axis = joint.find("axis")
                self._joints[joint.get("name") or ""] = (child, _floats(axis.get("xyz") if axis is not None else None, (1, 0, 0)))
        self.root: NodePath = next(n for name, n in self.links.items() if name not in children)

    def set_joint_angle(self, joint: str, radians: float) -> None:
        node, axis = self._joints[joint]
        q = LQuaternionf()
        q.setFromAxisAngleRad(radians, (axis[0], axis[1], axis[2]))
        node.setQuat(q)
