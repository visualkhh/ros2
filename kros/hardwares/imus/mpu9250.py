"""InvenSense MPU-9250 (TurtleBot3 OpenCR 보드의 IMU): 3축 자이로 + 3축 가속도계(+ 지자기). 여기서는 자이로/가속도계/방향만 다룬다."""
import math
import random

from kros.msgs.geometry_msgs import Quaternion, Vector3
from kros.msgs.sensor_msgs import Imu as ImuMessage
from kros.msgs.std_msgs import Header
from kros.hardwares.imus.imu import Imu
from kros.hardwares.noise import Noise
from kros.worlds.clock import Clock
from kros.hardwares.simulated_body import simulated_body

IMU_PERIOD = 0.01  # 초 (100Hz). 월드가 이보다 드물게 신호를 주면 신호마다 한 번이다


class Mpu9250(Imu):
    """MPU-9250의 공통부. 값을 만드는 sample()은 구현(시뮬레이션/더미/진짜)이 한다."""

    model = "MPU-9250"

    def __init__(self, frame_id: str = "imu_link", period: float | None = IMU_PERIOD) -> None:
        super().__init__(frame_id, period)


class SimulatedMpu9250(Mpu9250):
    """달려 있는 로봇(WorldObject)의 움직임에서 값을 만든다 (평면 운동만 있다):
    방향은 yaw만(roll/pitch 0), 각속도는 yaw 변화율(z축), 가속도는 속도 변화를 센서 축으로 돌린 것에 중력(월드의 중력, +z)을 더한 것.
    오차(기본은 없음): gyro/accel은 축마다 따로 뽑는 Noise, gyro_bias_walk(rad/s/sqrt(s))는 자이로 편향이 시간에 따라 천천히 떠도는 양이다.
    방향(yaw)은 진짜 IMU 보드처럼 자이로를 쌓아서 만든 것이라, 자이로 오차가 쌓여 방향이 서서히 틀어진다. seed가 같으면 같은 오차.
    첫 값이거나 꺼졌다 켜진 직후에는 변화율을 알 수 없어 가속도 0, 각속도 0으로 낸다.
    """

    def __init__(
        self, frame_id: str = "imu_link", period: float | None = IMU_PERIOD, gyro: Noise = Noise(), accel: Noise = Noise(), gyro_bias_walk: float = 0.0, seed: int | None = None
    ) -> None:
        super().__init__(frame_id, period)
        self.gyro, self.accel, self.gyro_bias_walk = gyro, accel, gyro_bias_walk
        self._rng = random.Random(seed)
        self._walk = 0.0  # 떠도는 자이로 편향
        self._yaw_error = 0.0  # 방향에 쌓인 오차
        self._last: tuple[float, float, float, float] | None = None  # 지난 값을 만들 때의 (시각, x, y, theta)
        self._previous_velocity: tuple[float, float] | None = None

    def sample(self, clock: Clock) -> ImuMessage:
        body, world = simulated_body(self)
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
        rng = self._rng
        self._walk += rng.gauss(0.0, self.gyro_bias_walk * math.sqrt(clock.dt))
        measured_z = self.gyro.apply(angular_z, rng) + self._walk
        self._yaw_error += (measured_z - angular_z) * clock.dt  # 방향은 측정한 각속도를 쌓은 것
        seen = yaw + self._yaw_error
        return ImuMessage(
            Header(clock.time, self.frame_id),
            Quaternion(0.0, 0.0, math.sin(seen / 2), math.cos(seen / 2)),
            Vector3(self.gyro.apply(0.0, rng) + self._walk, self.gyro.apply(0.0, rng) + self._walk, measured_z),
            Vector3(self.accel.apply(ax, rng), self.accel.apply(ay, rng), self.accel.apply(world.gravity, rng)),
        )


class DummyMpu9250(Mpu9250):
    """월드와 상관없이 가만히 서 있는 값(방향 없음, 각속도 0, 가속도는 중력 g만)을 보낸다 (월드 없이 파이프라인을 시험할 때)."""

    def __init__(self, gravity: float = 9.80665, frame_id: str = "imu_link", period: float | None = IMU_PERIOD) -> None:
        super().__init__(frame_id, period)
        self.gravity = gravity

    def sample(self, clock: Clock) -> ImuMessage:
        return ImuMessage(Header(clock.time, self.frame_id), Quaternion(), Vector3(), Vector3(0.0, 0.0, self.gravity))
