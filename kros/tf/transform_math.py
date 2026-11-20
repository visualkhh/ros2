"""3D 변환(이동 + 쿼터니언 회전) 계산. 표준 라이브러리만 쓰는 순수 함수이고 kros의 다른 부분을 모른다.

규약: Transform은 "자식 좌표계의 점을 부모 좌표계로 옮기는" 변환이다 (점_부모 = 회전 * 점_자식 + 이동).
쿼터니언은 (x, y, z, w), 회전은 오른손 법칙, w가 스칼라 부분이다.
"""
import math

from kros.msgs.geometry_msgs import Quaternion, Transform, Vector3

IDENTITY = Transform()


def quaternion_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm == 0.0:
        raise ValueError("크기가 0인 쿼터니언은 회전이 아니다")
    return Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    return Quaternion(-q.x, -q.y, -q.z, q.w)


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """a * b: 먼저 b로 돌리고 그 다음 a로 돌린 것과 같다."""
    return Quaternion(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def rotate(q: Quaternion, v: Vector3) -> Vector3:
    """쿼터니언 q로 벡터 v를 돌린다 (q * v * q^-1)."""
    r = quaternion_multiply(quaternion_multiply(q, Quaternion(v.x, v.y, v.z, 0.0)), quaternion_conjugate(q))
    return Vector3(r.x, r.y, r.z)


def from_axis_angle(axis: tuple[float, float, float], angle: float) -> Quaternion:
    """축(크기는 무시, 0이 아니어야 함)을 중심으로 angle(rad)만큼 도는 회전."""
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0:
        raise ValueError("회전축의 크기가 0이다")
    s = math.sin(angle / 2) / norm
    return Quaternion(x * s, y * s, z * s, math.cos(angle / 2))


def from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """URDF의 rpy: 고정축 x(roll), y(pitch), z(yaw) 순서로 돈 것 (= Rz(yaw) * Ry(pitch) * Rx(roll))."""
    qx = from_axis_angle((1, 0, 0), roll)
    qy = from_axis_angle((0, 1, 0), pitch)
    qz = from_axis_angle((0, 0, 1), yaw)
    return quaternion_multiply(qz, quaternion_multiply(qy, qx))


def compose(outer: Transform, inner: Transform) -> Transform:
    """outer * inner: inner(자식->중간) 다음에 outer(중간->부모)를 적용한 변환 (자식->부모)."""
    moved = rotate(outer.rotation, inner.translation)
    return Transform(
        Vector3(outer.translation.x + moved.x, outer.translation.y + moved.y, outer.translation.z + moved.z),
        quaternion_normalize(quaternion_multiply(outer.rotation, inner.rotation)),
    )


def inverse(t: Transform) -> Transform:
    """부모->자식 방향의 변환 (compose(t, inverse(t))는 항등)."""
    q = quaternion_conjugate(t.rotation)
    v = rotate(q, t.translation)
    return Transform(Vector3(-v.x, -v.y, -v.z), q)


def apply(t: Transform, point: Vector3) -> Vector3:
    """자식 좌표계의 점을 부모 좌표계로 옮긴다."""
    r = rotate(t.rotation, point)
    return Vector3(r.x + t.translation.x, r.y + t.translation.y, r.z + t.translation.z)


def slerp(a: Quaternion, b: Quaternion, ratio: float) -> Quaternion:
    """a에서 b로 가는 가장 짧은 호를 따라 ratio(0~1)만큼 간 회전 (구면 선형 보간)."""
    dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w
    if dot < 0.0:  # q와 -q는 같은 회전: 짧은 쪽으로
        b = Quaternion(-b.x, -b.y, -b.z, -b.w)
        dot = -dot
    if dot > 0.9995:  # 거의 같으면 선형 보간으로 충분하고 수치적으로 안전하다
        return quaternion_normalize(Quaternion(*(a_i + ratio * (b_i - a_i) for a_i, b_i in zip((a.x, a.y, a.z, a.w), (b.x, b.y, b.z, b.w)))))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    wa, wb = math.sin((1 - ratio) * theta) / sin_theta, math.sin(ratio * theta) / sin_theta
    return Quaternion(wa * a.x + wb * b.x, wa * a.y + wb * b.y, wa * a.z + wb * b.z, wa * a.w + wb * b.w)


def interpolate(a: Transform, b: Transform, ratio: float) -> Transform:
    """두 변환 사이 ratio(0~1) 지점: 이동은 선형, 회전은 구면 보간."""
    t = Vector3(*(a_i + ratio * (b_i - a_i) for a_i, b_i in zip((a.translation.x, a.translation.y, a.translation.z), (b.translation.x, b.translation.y, b.translation.z))))
    return Transform(t, slerp(a.rotation, b.rotation, ratio))
