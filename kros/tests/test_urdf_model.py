import struct
import tempfile
import unittest
from pathlib import Path

try:
    import panda3d  # noqa: F401  (examples/turtlebot3/requirements.txt)

    HAS_PANDA = True
except ImportError:
    HAS_PANDA = False

if HAS_PANDA:
    from kros.simulation.stl import load_stl
    from kros.simulation.urdf_model import UrdfModel

ROBOT = Path(__file__).parent.parent / "worlds" / "robots" / "turtlebot3" / "description"


@unittest.skipUnless(HAS_PANDA, "panda3d 없음")
class TestStl(unittest.TestCase):
    def test_binary_and_ascii_give_same_triangle(self) -> None:
        verts = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        with tempfile.TemporaryDirectory() as d:
            b, a = Path(d, "b.stl"), Path(d, "a.stl")
            b.write_bytes(b"\0" * 80 + struct.pack("<I", 1) + struct.pack("<12fH", 0, 0, 1, *verts, 0))
            a.write_text("solid t\nfacet normal 0 0 1\nouter loop\n" + "".join(
                f"vertex {verts[i]} {verts[i + 1]} {verts[i + 2]}\n" for i in (0, 3, 6)) + "endloop\nendfacet\nendsolid t\n")
            for path in (b, a):
                geom = load_stl(path, (2.0, 2.0, 2.0)).node().getGeom(0)
                self.assertEqual(geom.getVertexData().getNumRows(), 3)
                self.assertEqual(tuple(load_stl(path, (2.0, 2.0, 2.0)).getTightBounds()[1]), (2.0, 2.0, 0.0))


@unittest.skipUnless(HAS_PANDA, "panda3d 없음")
class TestUrdfModel(unittest.TestCase):
    def setUp(self) -> None:
        self.model = UrdfModel(ROBOT / "urdf" / "turtlebot3_waffle_pi.urdf", ROBOT)

    def test_links_and_root(self) -> None:
        self.assertEqual(self.model.root.getName(), "base_footprint")
        self.assertEqual({"base_link", "wheel_left_link", "wheel_right_link", "base_scan"} - set(self.model.links), set())

    def test_wheel_positions_match_urdf(self) -> None:
        for name, y in (("wheel_left_link", 0.144), ("wheel_right_link", -0.144)):
            p = self.model.links[name].getPos(self.model.root)
            self.assertAlmostEqual(p.y, y, places=5)
            self.assertAlmostEqual(p.z, 0.033, places=5)  # base_joint 0.010 + 조인트 0.023

    def test_positive_joint_angle_rolls_wheel_forward(self) -> None:
        wheel = self.model.links["wheel_left_link"]
        before = self.model.root.getRelativePoint(wheel, (0.033, 0, 0))  # 앞쪽 림 점
        self.model.set_joint_angle("wheel_left_joint", 0.3)
        after = self.model.root.getRelativePoint(wheel, (0.033, 0, 0))
        self.assertLess(after.z, before.z)  # 앞쪽이 아래로 말림 = 전진 구름

    def test_unsupported_visual_raises(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            urdf = Path(d, "r.urdf")
            urdf.write_text('<robot name="r"><link name="l"><visual><geometry><box size="1 1 1"/></geometry></visual></link></robot>')
            with self.assertRaises(NotImplementedError):
                UrdfModel(urdf, Path(d))
