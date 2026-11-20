import math

from kros.msgs.geometry_msgs import Pose2D, Twist
from kros.worlds.clock import Clock
from kros.hardwares import Actuator, Motor
from kros.worlds.world import World
from kros.worlds.objects.world_object import WorldObject

MAX_SUBSTEP = 0.05  # m, 한 번에 크게 움직이면 벽을 통째로 건너뛰므로 이 이하로 쪼갠다


class DiffDrive(Actuator[Twist]):
    """TurtleBot3의 구동부: 바퀴 모터 둘(left, right)로 차동구동한다. write(Twist)로 몸체 속도 명령을 내리면
    바퀴 속도로 바꿔 모터에 주고(역기구학), 모터가 실제로 돈 속도로 몸체를 움직인다(정기구학). 환경(벽)에 닿으면 world가 밀어낸다.
    다른 객체와의 충돌은 world가 step에서 푼다. 바퀴와 몸체 크기는 이 로봇의 것이고, 모터는 부품(hardwares)이다."""

    def __init__(self, left: Motor, right: Motor, wheel_radius: float, wheel_separation: float, body_radius: float) -> None:
        super().__init__(Twist())
        self.left, self.right = left, right
        self.wheel_radius = wheel_radius  # m
        self.wheel_separation = wheel_separation  # m, 두 바퀴 사이
        self.body_radius = body_radius  # m, 몸체를 원으로 본 충돌 반지름
        self.add(left, "left_wheel")
        self.add(right, "right_wheel")

    def apply(self, command: Twist, clock: Clock, world: World) -> None:
        """역기구학: 몸체 속도 명령 -> 바퀴 속도 목표."""
        v, w, r, half = command.linear.x, command.angular.z, self.wheel_radius, self.wheel_separation / 2
        self.left.write((v - w * half) / r)
        self.right.write((v + w * half) / r)

    def tick(self, clock: Clock, world: World) -> None:
        super().tick(clock, world)  # 목표를 바퀴에 주고(apply), 바퀴 모터들이 돈다
        body = self.owner
        assert isinstance(body, WorldObject), "DiffDrive는 WorldObject(로봇)에 달아야 한다"
        if self.is_on:
            body.pose = self.move(body.pose, clock.dt, world)

    def wheel_velocity(self, motor: Motor) -> float:
        return motor.velocity if motor.is_on else 0.0

    def move(self, pose: Pose2D, dt: float, world: World) -> Pose2D:
        """정기구학: 바퀴가 실제로 돈 속도로 dt초 동안 움직인 새 위치."""
        wl, wr = self.wheel_velocity(self.left), self.wheel_velocity(self.right)
        v = self.wheel_radius * (wl + wr) / 2
        w = self.wheel_radius * (wr - wl) / self.wheel_separation
        n = max(1, math.ceil(abs(v) * dt / MAX_SUBSTEP))
        h = dt / n
        for _ in range(n):
            mid = pose.theta + w * h / 2
            x, y = world.push_out(pose.x + v * math.cos(mid) * h, pose.y + v * math.sin(mid) * h, self.body_radius)
            pose = Pose2D(x, y, (pose.theta + w * h + math.pi) % (2 * math.pi) - math.pi)
        return pose
