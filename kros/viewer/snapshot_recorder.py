from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.world import World

from .snapshot import Snapshot


class SnapshotRecorder:
    """월드가 스텝을 끝낼 때마다(SimulationRunner의 리스너) 그 순간을 Snapshot으로 만들어 latest로 들고 있는다. 화면은 latest만 읽는다.

    월드와 로봇(TurtleBot3)을 아는 곳은 여기뿐이다 (러너는 월드만 안다). record는 시뮬레이션 스레드에서 불리고,
    latest는 한 줄 대입이라 읽는 쪽(화면)이 중간 값을 볼 일이 없다.
    """

    def __init__(self, world: World, robot: TurtleBot3) -> None:
        self.world, self.robot = world, robot
        self.latest: Snapshot = self.snapshot()

    def record(self, world: World) -> None:
        self.latest = self.snapshot()

    def snapshot(self) -> Snapshot:
        objects = self.world.objects
        return Snapshot(
            self.world.time,
            tuple(o.pose for o in objects),
            objects.index(self.robot),
            self.robot.command,
            (self.robot.drive.left.position, self.robot.drive.right.position),
            self.robot.lidar.latest,
            self.robot.lidar.scan_pose,
        )
