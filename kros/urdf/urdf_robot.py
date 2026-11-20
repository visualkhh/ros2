import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass

JOINT_TYPES = ("fixed", "continuous", "revolute", "prismatic", "floating", "planar")


class UrdfError(ValueError):
    """URDF를 읽을 수 없거나 로봇 구조(링크/관절 트리)가 올바르지 않다."""


@dataclass(frozen=True)
class UrdfJoint:
    """URDF 관절 하나: 부모 링크에서 본 자식 링크의 기준 위치(xyz, rpy)와, 움직이는 관절이면 움직이는 축."""

    name: str
    type: str
    parent: str
    child: str
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0)

    @property
    def is_movable(self) -> bool:
        return self.type in ("continuous", "revolute", "prismatic")


@dataclass(frozen=True)
class UrdfRobot:
    name: str
    root: str  # 뿌리 링크 (부모가 없는 링크)
    links: tuple[str, ...]
    joints: tuple[UrdfJoint, ...]


def _floats(text: str | None, default: tuple[float, float, float], where: str) -> tuple[float, float, float]:
    if text is None:
        return default
    parts = text.split()
    try:
        values = tuple(float(p) for p in parts)
    except ValueError:
        raise UrdfError(f"{where}: 숫자가 아니다: {text!r}") from None
    if len(values) != 3:
        raise UrdfError(f"{where}: 숫자 3개여야 한다: {text!r}")
    return values[0], values[1], values[2]


def parse_urdf(text: str, xacro_args: Mapping[str, str] | None = None) -> UrdfRobot:
    """URDF 본문을 읽어 링크/관절 트리를 만든다 (표준 라이브러리만). 구조를 검증한다: 이름 중복, 없는 링크, 부모가 둘, 뿌리가 둘 이상,
    뿌리에서 닿지 않는 링크가 있으면 UrdfError.

    xacro 파일은 인자 치환(`${이름}`)만 지원한다: xacro_args(기본 {"namespace": ""})에 있는 것만 바꾼다. 그 밖의 xacro 기능은 지원하지 않고,
    치환 못 한 `${...}`가 이름에 남으면 UrdfError다.
    """
    args = {"namespace": ""} if xacro_args is None else dict(xacro_args)
    for key, value in args.items():
        text = text.replace("${" + key + "}", value)
    try:
        root_element = ET.fromstring(text)
    except ET.ParseError as error:
        raise UrdfError(f"XML을 읽을 수 없다: {error}") from None
    if root_element.tag != "robot":
        raise UrdfError(f"최상위 태그가 <robot>이 아니다: <{root_element.tag}>")
    links: list[str] = []
    for link in root_element.findall("link"):
        name = link.get("name")
        if not name:
            raise UrdfError("이름 없는 <link>가 있다")
        if name in links:
            raise UrdfError(f"링크 이름이 겹친다: {name!r}")
        links.append(name)
    joints: list[UrdfJoint] = []
    for joint in root_element.findall("joint"):
        name, kind = joint.get("name"), joint.get("type")
        if not name:
            raise UrdfError("이름 없는 <joint>가 있다")
        if kind not in JOINT_TYPES:
            raise UrdfError(f"관절 {name!r}의 type이 올바르지 않다: {kind!r}")
        parent_element, child_element = joint.find("parent"), joint.find("child")
        parent = parent_element.get("link") if parent_element is not None else None
        child = child_element.get("link") if child_element is not None else None
        if not parent or not child:
            raise UrdfError(f"관절 {name!r}에 <parent link=...>/<child link=...>가 없다")
        origin, axis = joint.find("origin"), joint.find("axis")
        joints.append(
            UrdfJoint(
                name,
                kind,
                parent,
                child,
                _floats(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0), f"관절 {name!r}의 origin xyz"),
                _floats(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0), f"관절 {name!r}의 origin rpy"),
                _floats(axis.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0), f"관절 {name!r}의 axis"),
            )
        )
    for text_name in [j.name for j in joints] + links:
        if "${" in text_name:
            raise UrdfError(f"치환하지 못한 xacro 표현이 이름에 남았다: {text_name!r} (지원하는 인자: {sorted(args)})")
    return _validate(root_element.get("name") or "", links, joints)


def _validate(name: str, links: list[str], joints: list[UrdfJoint]) -> UrdfRobot:
    if not links:
        raise UrdfError("<link>가 하나도 없다")
    seen: set[str] = set()
    parent_of: dict[str, str] = {}
    for joint in joints:
        if joint.name in seen:
            raise UrdfError(f"관절 이름이 겹친다: {joint.name!r}")
        seen.add(joint.name)
        for link in (joint.parent, joint.child):
            if link not in links:
                raise UrdfError(f"관절 {joint.name!r}이 없는 링크를 가리킨다: {link!r}")
        if joint.child in parent_of:
            raise UrdfError(f"링크 {joint.child!r}의 부모가 둘 이상이다 ({parent_of[joint.child]!r}, {joint.parent!r})")
        parent_of[joint.child] = joint.parent
    roots = [link for link in links if link not in parent_of]
    if len(roots) != 1:
        raise UrdfError(f"뿌리 링크(부모가 없는 링크)가 정확히 하나여야 한다: {roots}")
    for link in links:  # 모든 링크가 뿌리에 닿아야 한다 (순환이 있으면 닿지 못한다)
        node, hops = link, 0
        while node in parent_of:
            node, hops = parent_of[node], hops + 1
            if hops > len(links):
                raise UrdfError(f"링크 {link!r}에서 관절 순환이 있다")
        if node != roots[0]:
            raise UrdfError(f"링크 {link!r}이 뿌리 {roots[0]!r}에 이어져 있지 않다")
    return UrdfRobot(name, roots[0], tuple(links), tuple(joints))
