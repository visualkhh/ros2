import tempfile
import time
import unittest
from pathlib import Path

from kros import Kros
from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose
from kros.msgs.nav_msgs import FREE, OCCUPIED, MapMetaData, OccupancyGrid
from kros.msgs.std_msgs import Header, String
from kros.nodes import Cartographer, MapCatalog, MapServer, Node
from kros.nodes.map_server import load_map, save_map
from kros.nodes.map_server.map_catalog import MAP_STATUS_TOPIC
from kros.timer import SimulatedTimeSource
from kros.topic import QOS_LATCHED


def grid() -> OccupancyGrid:
    return OccupancyGrid(Header(0.0, "map"), MapMetaData(0.0, 0.05, 2, 2, Pose(Point(0.0, 0.0, 0.0))), (OCCUPIED, FREE, FREE, FREE))


class TestMapCatalog(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.maps = Path(directory.name)
        save_map(grid(), self.maps / "alpha" / "map", 0.196, 0.65)
        save_map(grid(), self.maps / "beta" / "map", 0.196, 0.65)
        folder = {"maps_directory": str(self.maps)}
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["user"] = Node(ctx, "User")
            return [made["user"], MapServer(ctx, parameters=folder), Cartographer(ctx, parameters={"mode": "idle"}), MapCatalog(ctx, parameters={**folder, "list_period": 0.05})]

        self.source = SimulatedTimeSource()
        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        self.user = made["user"]
        self.statuses: list[str] = []
        self.maps_seen: list[OccupancyGrid] = []
        self.lists: list[list[str]] = []
        self.user.subscribe(MAP_STATUS_TOPIC, String, lambda message: self.statuses.append(message.data), QOS_LATCHED)
        self.user.subscribe("map_list", String, lambda message: self.lists.append(message.data.splitlines()), QOS_LATCHED)
        self.user.subscribe("map", OccupancyGrid, self.maps_seen.append, QOS_LATCHED)
        self.advance(0.3)

    def advance(self, seconds: float) -> None:
        for _ in range(round(seconds / 0.02)):
            self.source.set(self.source.now() + 0.02)
            self.kros.context.timer_manager.join()
        self.kros.context.topic_executor.join()

    def test_lists_maps_from_the_directory(self) -> None:
        self.assertEqual(self.lists[-1], ["alpha", "beta"])

    def test_map_change_loads_and_empty_change_deletes(self) -> None:
        self.user.publish("map_change", String("alpha"))
        self.advance(0.3)
        self.assertEqual(self.statuses[-1], "loaded alpha")
        self.assertEqual(str(self.kros.context.parameter_manager.get("cartographer", "mode")), "localization")
        self.assertGreater(self.maps_seen[-1].info.width, 0)
        self.assertTrue((self.maps / "alpha" / "map.yaml").exists())

        self.user.publish("map_change", String(""))
        self.advance(0.3)
        self.assertEqual(self.statuses[-1], "deleted alpha")
        self.assertFalse((self.maps / "alpha").exists())  # 폴더째 지운다
        self.assertTrue((self.maps / "beta" / "map.yaml").exists())
        self.assertEqual((self.maps_seen[-1].info.width, self.maps_seen[-1].info.height), (0, 0))  # 지도 내림

    def test_empty_change_without_a_loaded_map_does_nothing(self) -> None:
        self.user.publish("map_change", String(""))
        self.advance(0.3)
        self.assertIn("no map loaded", self.statuses[-1])
        self.assertTrue((self.maps / "alpha").exists())

    def test_invalid_names_are_rejected(self) -> None:
        for name in ("../alpha", "/etc", ".."):
            self.user.publish("map_change", String(name))
            self.advance(0.2)
            self.assertIn("invalid map name", self.statuses[-1])
        self.assertTrue((self.maps / "alpha").exists())

    def test_loaded_map_is_replaced_by_another(self) -> None:
        self.user.publish("map_change", String("alpha"))
        self.advance(0.2)
        self.user.publish("map_change", String("beta"))
        self.advance(0.2)
        self.assertEqual(self.statuses[-1], "loaded beta")
        self.assertTrue((self.maps / "alpha").exists())  # 바꾸기는 지우지 않는다
        self.assertTrue((self.maps / "beta").exists())


if __name__ == "__main__":
    unittest.main()
