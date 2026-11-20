import math

from kros.msgs.geometry_msgs import Pose2D, Twist
from kros.worlds.clock import Clock
from kros.hardwares import Actuator, Motor
from kros.hardwares.simulated_body import simulated_body
from kros.worlds.world import World

MAX_SUBSTEP = 0.05  # m, 한 번에 크게 움직이면 벽을 통째로 건너뛰므로 이 이하로 쪼갠다


def wheel_speeds(command: Twist, wheel_radius: float, wheel_separation: float, max_wheel: float | None = None) -> tuple[float, float]:
    """역기구학: 몸체 속도 명령 -> (왼쪽, 오른쪽) 바퀴 속도 rad/s. max_wheel을 주면 한쪽 바퀴가 그걸 넘을 때 두 바퀴를 같은 비율로 줄인다
    (바퀴마다 따로 자르면 빠른 호가 안쪽 바퀴만 도는 제자리 회전이 된다)."""
    v, w, half = command.linear.x, command.angular.z, wheel_separation / 2
    left, right = (v - w * half) / wheel_radius, (v + w * half) / wheel_radius
    peak = max(abs(left), abs(right))
    if max_wheel is not None and peak > max_wheel:
        left, right = left * max_wheel / peak, right * max_wheel / peak
    return left, right


class DiffDrive(Actuator[Twist]):
    """차동구동 구동부 (TurtleBot3의 OpenCR가 쓴다): 바퀴 모터 둘(left, right)로 차동구동한다. write(Twist)로 몸체 속도 명령을 내리면
    바퀴 속도로 바꿔 모터에 주고(역기구학), 모터가 실제로 돈 속도로 몸체를 움직인다(정기구학). 환경(벽)에 닿으면 world가 밀어낸다.
    다른 객체와의 충돌은 world가 step에서 푼다. 바퀴와 몸체 크기는 로봇의 것이고, 모터는 부품(hardwares)이다."""

    def __init__(self, left: Motor, right: Motor, wheel_radius: float, wheel_separation: float, body_radius: float) -> None:
        super().__init__(Twist())
        self.left, self.right = left, right
        self.wheel_radius = wheel_radius  # m
        self.wheel_separation = wheel_separation  # m, 두 바퀴 사이
        self.body_radius = body_radius  # m, 몸체를 원으로 본 충돌 반지름
        self.add(left, "left_wheel")
        self.add(right, "right_wheel")

    def apply(self, command: Twist, clock: Clock) -> None:
        """역기구학: 몸체 속도 명령 -> 바퀴 속도 목표."""
        left, right = wheel_speeds(command, self.wheel_radius, self.wheel_separation)
        self.left.write(left)
        self.right.write(right)

    def tick(self, clock: Clock) -> None:
        super().tick(clock)  # 목표를 바퀴에 주고(apply), 바퀴 모터들이 돈다
        if self.is_on:
            body, world = simulated_body(self)  # 시뮬레이션: 몸체를 월드 안에서 움직인다
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
