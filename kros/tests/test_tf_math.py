import math
import unittest

from kros.msgs.geometry_msgs import Quaternion, Transform, Vector3
from kros.tf.transform_math import (
    IDENTITY,
    apply,
    compose,
    from_axis_angle,
    from_rpy,
    interpolate,
    inverse,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_normalize,
    rotate,
    slerp,
)


def close(a: Vector3, x: float, y: float, z: float, places: int = 9) -> bool:
    return all(abs(p - q) < 10**-places for p, q in ((a.x, x), (a.y, y), (a.z, z)))


def quaternion_close(a: Quaternion, b: Quaternion) -> bool:
    same = all(abs(p - q) < 1e-9 for p, q in zip((a.x, a.y, a.z, a.w), (b.x, b.y, b.z, b.w)))
    flipped = all(abs(p + q) < 1e-9 for p, q in zip((a.x, a.y, a.z, a.w), (b.x, b.y, b.z, b.w)))  # q와 -q는 같은 회전
    return same or flipped


def rotation_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    """쿼터니언을 쓰지 않고 행렬로 따로 계산한 Rz(yaw) * Ry(pitch) * Rx(roll) (검증용 독립 구현)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


class TestQuaternion(unittest.TestCase):
    def test_rotating_by_90_degrees_about_z(self) -> None:
        q = from_axis_angle((0, 0, 1), math.pi / 2)
        self.assertTrue(close(rotate(q, Vector3(1, 0, 0)), 0, 1, 0))  # x축이 y축으로
        self.assertTrue(close(rotate(q, Vector3(0, 1, 0)), -1, 0, 0))
        self.assertTrue(close(rotate(q, Vector3(0, 0, 1)), 0, 0, 1))  # 축 위의 점은 안 움직인다

    def test_right_hand_rule_for_each_axis(self) -> None:
        self.assertTrue(close(rotate(from_axis_angle((1, 0, 0), math.pi / 2), Vector3(0, 1, 0)), 0, 0, 1))  # x축: y -> z
        self.assertTrue(close(rotate(from_axis_angle((0, 1, 0), math.pi / 2), Vector3(0, 0, 1)), 1, 0, 0))  # y축: z -> x

    def test_axis_does_not_have_to_be_unit_length(self) -> None:
        self.assertTrue(quaternion_close(from_axis_angle((0, 0, 5), 1.0), from_axis_angle((0, 0, 1), 1.0)))
        with self.assertRaises(ValueError):
            from_axis_angle((0, 0, 0), 1.0)

    def test_multiply_means_rotate_by_b_first_then_a(self) -> None:
        a, b = from_axis_angle((0, 0, 1), math.pi / 2), from_axis_angle((1, 0, 0), math.pi / 2)
        v = Vector3(0, 1, 0)
        self.assertTrue(close(rotate(quaternion_multiply(a, b), v), *(lambda r: (r.x, r.y, r.z))(rotate(a, rotate(b, v)))))
        # 순서가 다르면 결과도 다르다 (회전은 교환되지 않는다)
        self.assertFalse(quaternion_close(quaternion_multiply(a, b), quaternion_multiply(b, a)))

    def test_conjugate_undoes_the_rotation(self) -> None:
        q = from_axis_angle((1, 2, 3), 0.7)
        self.assertTrue(quaternion_close(quaternion_multiply(q, quaternion_conjugate(q)), Quaternion()))

    def test_normalize_and_zero_quaternion(self) -> None:
        q = quaternion_normalize(Quaternion(0, 0, 2, 2))
        self.assertAlmostEqual(math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2), 1.0)
        with self.assertRaises(ValueError):
            quaternion_normalize(Quaternion(0, 0, 0, 0))


class TestRpy(unittest.TestCase):
    def test_rpy_matches_an_independent_matrix_computation(self) -> None:
        for roll, pitch, yaw in [(0.3, -0.5, 1.2), (-1.57, 0, 0), (1.0, 1.0, 1.0), (0, 0.4, -2.5), (3.0, -0.2, 0.1)]:
            q, m = from_rpy(roll, pitch, yaw), rotation_matrix(roll, pitch, yaw)
            for i, unit in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1))):
                rotated = rotate(q, Vector3(*unit))
                expected = (m[0][i], m[1][i], m[2][i])  # 행렬의 i번째 열
                self.assertTrue(close(rotated, *expected), (roll, pitch, yaw, unit))

    def test_urdf_wheel_orientation(self) -> None:
        """TurtleBot3 바퀴는 rpy=(-1.57, 0, 0): 바퀴의 z축(회전축)이 로봇의 +y축을 향한다. Rx(a)(0,0,1) = (0, -sin a, cos a)
        (1.57은 정확한 pi/2가 아니라서 z에 cos(1.57)=0.0008이 남는다)."""
        a = -1.57
        self.assertTrue(close(rotate(from_rpy(a, 0, 0), Vector3(0, 0, 1)), 0, -math.sin(a), math.cos(a)))


class TestTransform(unittest.TestCase):
    def test_compose_applies_inner_first_then_outer(self) -> None:
        outer = Transform(Vector3(1, 0, 0), from_axis_angle((0, 0, 1), math.pi / 2))
        inner = Transform(Vector3(1, 0, 0))
        result = compose(outer, inner)
        self.assertTrue(close(result.translation, 1, 1, 0))  # inner의 (1,0,0)이 outer에 의해 (0,1,0)이 되고 outer 이동 (1,0,0)이 더해진다
        self.assertTrue(close(apply(result, Vector3(0, 0, 0)), 1, 1, 0))

    def test_compose_equals_applying_one_after_another(self) -> None:
        a = Transform(Vector3(0.3, -1, 2), from_rpy(0.2, 0.4, -0.6))
        b = Transform(Vector3(-2, 0.5, 1), from_rpy(-0.3, 0.1, 1.1))
        p = Vector3(0.7, -0.2, 1.5)
        direct = apply(a, apply(b, p))
        via = apply(compose(a, b), p)
        self.assertTrue(close(direct, via.x, via.y, via.z))

    def test_inverse_undoes_the_transform(self) -> None:
        t = Transform(Vector3(0.3, -1, 2), from_rpy(0.2, 0.4, -0.6))
        p = Vector3(0.7, -0.2, 1.5)
        back = apply(inverse(t), apply(t, p))
        self.assertTrue(close(back, 0.7, -0.2, 1.5))
        identity = compose(t, inverse(t))
        self.assertTrue(close(identity.translation, 0, 0, 0))
        self.assertTrue(quaternion_close(identity.rotation, Quaternion()))

    def test_identity_is_neutral(self) -> None:
        t = Transform(Vector3(1, 2, 3), from_rpy(0.1, 0.2, 0.3))
        for result in (compose(IDENTITY, t), compose(t, IDENTITY)):
            self.assertTrue(close(result.translation, 1, 2, 3))
            self.assertTrue(quaternion_close(result.rotation, t.rotation))


class TestInterpolation(unittest.TestCase):
    def test_slerp_halfway_is_half_the_angle(self) -> None:
        a, b = Quaternion(), from_axis_angle((0, 0, 1), math.pi / 2)
        self.assertTrue(quaternion_close(slerp(a, b, 0.5), from_axis_angle((0, 0, 1), math.pi / 4)))
        self.assertTrue(quaternion_close(slerp(a, b, 0.0), a))
        self.assertTrue(quaternion_close(slerp(a, b, 1.0), b))

    def test_slerp_takes_the_short_way_around(self) -> None:
        a = from_axis_angle((0, 0, 1), math.radians(170))
        b = from_axis_angle((0, 0, 1), math.radians(-170))  # 170도와 -170도는 20도 차이다 (340도가 아니다)
        mid = slerp(a, b, 0.5)
        self.assertTrue(quaternion_close(mid, from_axis_angle((0, 0, 1), math.radians(180))))

    def test_slerp_of_nearly_equal_rotations_is_stable(self) -> None:
        a = from_axis_angle((0, 0, 1), 0.5)
        mid = slerp(a, a, 0.5)
        self.assertTrue(quaternion_close(mid, a))

    def test_interpolate_translation_linear_and_rotation_spherical(self) -> None:
        a = Transform(Vector3(0, 0, 0), Quaternion())
        b = Transform(Vector3(2, 4, -2), from_axis_angle((0, 0, 1), math.pi / 2))
        mid = interpolate(a, b, 0.25)
        self.assertTrue(close(mid.translation, 0.5, 1.0, -0.5))
        self.assertTrue(quaternion_close(mid.rotation, from_axis_angle((0, 0, 1), math.pi / 8)))


if __name__ == "__main__":
    unittest.main()
