import unittest

from kros.urdf import UrdfError, parse_urdf
from kros.worlds.robots.turtlebot3 import TurtleBot3

MINIMAL = """<robot name="r"><link name="a"/><link name="b"/><link name="c"/>
<joint name="ab" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 2 3" rpy="0 0 1.5"/></joint>
<joint name="bc" type="continuous"><parent link="b"/><child link="c"/><axis xyz="0 0 1"/></joint></robot>"""


class TestParse(unittest.TestCase):
    def test_minimal_robot(self) -> None:
        robot = parse_urdf(MINIMAL)
        self.assertEqual((robot.name, robot.root, robot.links), ("r", "a", ("a", "b", "c")))
        ab, bc = robot.joints
        self.assertEqual((ab.type, ab.parent, ab.child, ab.xyz, ab.rpy), ("fixed", "a", "b", (1.0, 2.0, 3.0), (0.0, 0.0, 1.5)))
        self.assertEqual((bc.is_movable, bc.axis, bc.xyz), (True, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)))  # origin 없으면 0
        self.assertFalse(ab.is_movable)

    def test_axis_defaults_to_x(self) -> None:
        robot = parse_urdf('<robot name="r"><link name="a"/><link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint></robot>')
        self.assertEqual(robot.joints[0].axis, (1.0, 0.0, 0.0))

    def test_turtlebot3_waffle_pi_urdf(self) -> None:
        robot = parse_urdf(TurtleBot3.urdf_path.read_text())
        self.assertEqual((robot.name, robot.root), ("turtlebot3_waffle_pi", "base_footprint"))
        joints = {j.name: j for j in robot.joints}
        self.assertEqual({n for n, j in joints.items() if j.is_movable}, {"wheel_left_joint", "wheel_right_joint"})
        self.assertEqual(joints["wheel_left_joint"].type, "continuous")
        self.assertEqual(joints["wheel_left_joint"].axis, (0.0, 0.0, 1.0))
        self.assertEqual(joints["scan_joint"].xyz, (-0.064, 0.0, 0.122))
        self.assertIn("base_scan", robot.links)  # ${namespace}가 빈 값으로 치환됐다

    def test_xacro_namespace_argument_is_substituted(self) -> None:
        text = '<robot name="r"><link name="${namespace}a"/></robot>'
        self.assertEqual(parse_urdf(text).links, ("a",))
        self.assertEqual(parse_urdf(text, {"namespace": "bot1/"}).links, ("bot1/a",))

    def test_unsupported_xacro_expression_is_an_error_not_a_silent_wrong_name(self) -> None:
        with self.assertRaisesRegex(UrdfError, "xacro"):
            parse_urdf('<robot name="r"><link name="${prefix}a"/></robot>')


class TestValidation(unittest.TestCase):
    def check(self, body: str, message: str) -> None:
        with self.assertRaisesRegex(UrdfError, message):
            parse_urdf(f'<robot name="r">{body}</robot>')

    def test_not_xml_and_wrong_root(self) -> None:
        with self.assertRaises(UrdfError):
            parse_urdf("<robot")
        with self.assertRaisesRegex(UrdfError, "robot"):
            parse_urdf("<mesh/>")

    def test_no_links(self) -> None:
        self.check("", "가 하나도 없다")

    def test_duplicate_names(self) -> None:
        self.check('<link name="a"/><link name="a"/>', "링크 이름이 겹친다")
        self.check(
            '<link name="a"/><link name="b"/><link name="c"/>'
            '<joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>'
            '<joint name="j" type="fixed"><parent link="b"/><child link="c"/></joint>',
            "관절 이름이 겹친다",
        )

    def test_joint_pointing_to_a_missing_link(self) -> None:
        self.check('<link name="a"/><joint name="j" type="fixed"><parent link="a"/><child link="zzz"/></joint>', "없는 링크")

    def test_joint_without_parent_or_child_and_bad_type(self) -> None:
        self.check('<link name="a"/><joint name="j" type="fixed"><child link="a"/></joint>', "parent")
        self.check('<link name="a"/><link name="b"/><joint name="j" type="magic"><parent link="a"/><child link="b"/></joint>', "type")

    def test_link_with_two_parents(self) -> None:
        self.check(
            '<link name="a"/><link name="b"/><link name="c"/>'
            '<joint name="j1" type="fixed"><parent link="a"/><child link="c"/></joint>'
            '<joint name="j2" type="fixed"><parent link="b"/><child link="c"/></joint>',
            "부모가 둘",
        )

    def test_more_than_one_root(self) -> None:
        self.check('<link name="a"/><link name="b"/>', "뿌리")

    def test_cycle_is_rejected(self) -> None:
        self.check(
            '<link name="r"/><link name="a"/><link name="b"/>'
            '<joint name="j1" type="fixed"><parent link="a"/><child link="b"/></joint>'
            '<joint name="j2" type="fixed"><parent link="b"/><child link="a"/></joint>',
            "순환|뿌리",
        )

    def test_bad_numbers(self) -> None:
        self.check('<link name="a"/><link name="b"/><joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 2"/></joint>', "숫자 3개")
        self.check('<link name="a"/><link name="b"/><joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="a b c"/></joint>', "숫자가 아니다")


if __name__ == "__main__":
    unittest.main()
