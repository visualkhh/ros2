"""InvenSense MPU-9250 (TurtleBot3 OpenCR 보드의 IMU): 3축 자이로 + 3축 가속도계(+ 지자기). 여기서는 자이로/가속도계/방향만 다룬다."""
import math

from kros.msgs.geometry_msgs import Quaternion, Vector3
from kros.msgs.sensor_msgs import Imu as ImuMessage
from kros.msgs.std_msgs import Header
from kros.hardwares.imus.imu import Imu
from kros.worlds.clock import Clock
from kros.worlds.objects.world_object import WorldObject
from kros.worlds.world import World

IMU_PERIOD = 0.01  # 초 (100Hz). 월드가 이보다 드물게 신호를 주면 신호마다 한 번이다


class Mpu9250(Imu):
    """MPU-9250의 공통부. 값을 만드는 sample()은 구현(시뮬레이션/더미/진짜)이 한다."""

    model = "MPU-9250"

    def __init__(self, frame_id: str = "imu_link", period: float | None = IMU_PERIOD) -> None:
        super().__init__(frame_id, period)


class SimulatedMpu9250(Mpu9250):
    """달려 있는 로봇(WorldObject)의 움직임에서 값을 만든다 (평면 운동만 있다):
    방향은 yaw만(roll/pitch 0), 각속도는 yaw 변화율(z축), 가속도는 속도 변화를 센서 축으로 돌린 것에 중력(월드의 중력, +z)을 더한 것.
    노이즈와 바이어스는 아직 없다. 첫 값이거나 꺼졌다 켜진 직후에는 변화율을 알 수 없어 가속도 0, 각속도 0으로 낸다.
    """

    def __init__(self, frame_id: str = "imu_link", period: float | None = IMU_PERIOD) -> None:
        super().__init__(frame_id, period)
        self._last: tuple[float, float, float, float] | None = None  # 지난 값을 만들 때의 (시각, x, y, theta)
        self._previous_velocity: tuple[float, float] | None = None

    def sample(self, clock: Clock, world: World) -> ImuMessage:
        body = self.owner
        assert isinstance(body, WorldObject), "SimulatedMpu9250은 WorldObject(로봇)에 달아야 한다"
        pose = body.pose
        yaw, c, s = pose.theta, math.cos(pose.theta), math.sin(pose.theta)
        angular_z, ax, ay = 0.0, 0.0, 0.0
        last = self._last
        if last is not None and 0 < clock.time - last[0] <= clock.dt * 1.5 + 1e-9:  # 연속된 값일 때만 변화율을 구한다
            dt = clock.time - last[0]
            velocity = ((pose.x - last[1]) / dt, (pose.y - last[2]) / dt)
            angular_z = math.atan2(math.sin(yaw - last[3]), math.cos(yaw - last[3])) / dt  # -pi~pi로 감아서 차이를 구한다
            if self._previous_velocity is not None:
                wx, wy = (velocity[0] - self._previous_velocity[0]) / dt, (velocity[1] - self._previous_velocity[1]) / dt
                ax, ay = c * wx + s * wy, -s * wx + c * wy  # 월드 가속도를 센서 축으로
            self._previous_velocity = velocity
        else:
            self._previous_velocity = None
        self._last = (clock.time, pose.x, pose.y, yaw)
        return ImuMessage(
            Header(clock.time, self.frame_id), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), Vector3(0.0, 0.0, angular_z), Vector3(ax, ay, world.gravity)
        )


class DummyMpu9250(Mpu9250):
    """월드와 상관없이 가만히 서 있는 값(방향 없음, 각속도 0, 가속도는 중력 g만)을 보낸다 (월드 없이 파이프라인을 시험할 때)."""

    def __init__(self, gravity: float = 9.80665, frame_id: str = "imu_link", period: float | None = IMU_PERIOD) -> None:
        super().__init__(frame_id, period)
        self.gravity = gravity

    def sample(self, clock: Clock, world: World) -> ImuMessage:
        return ImuMessage(Header(clock.time, self.frame_id), Quaternion(), Vector3(), Vector3(0.0, 0.0, self.gravity))
