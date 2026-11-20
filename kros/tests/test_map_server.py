import math
import tempfile
import unittest
from pathlib import Path

from kros import Kros
from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose, Quaternion
from kros.msgs.nav2_msgs import LoadMap, LoadMapRequest, LoadMapResponse, SaveMap, SaveMapRequest
from kros.msgs.nav_msgs import FREE, OCCUPIED, UNKNOWN, MapMetaData, OccupancyGrid
from kros.msgs.std_msgs import Header
from kros.nodes import MapSaver, MapServer, Node
from kros.nodes.map_server import MapIoError, load_map, save_map
from kros.timer import SimulatedTimeSource
from kros.topic import QOS_LATCHED

YAW = 0.3


def grid() -> OccupancyGrid:
    """3x2 지도: 아래 줄(y=0) [벽, 빈 곳, 모름], 윗줄(y=1) [빈 곳, 50(애매), 벽]. 원점 (1, -2), 0.3rad 돌아 있다."""
    origin = Pose(Point(1.0, -2.0, 0.0), Quaternion(0.0, 0.0, math.sin(YAW / 2), math.cos(YAW / 2)))
    return OccupancyGrid(Header(0.0, "map"), MapMetaData(0.0, 0.05, 3, 2, origin), (OCCUPIED, FREE, UNKNOWN, FREE, 50, OCCUPIED))


class TestMapIo(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.dir = Path(directory.name)

    def test_saves_pgm_and_yaml_like_ros_map_saver(self) -> None:
        yaml = save_map(grid(), self.dir / "sub" / "room")
        self.assertEqual(yaml, self.dir / "sub" / "room.yaml")
        text = yaml.read_text()
        for line in ("image: room.pgm", "mode: trinary", "resolution: 0.05", "negate: 0", "occupied_thresh: 0.65", "free_thresh: 0.196"):
            self.assertIn(line, text)
        self.assertIn("origin: [1.0, -2.0, 0.3", text)  # [x, y, yaw]
        data = (self.dir / "sub" / "room.pgm").read_bytes()
        self.assertTrue(data.startswith(b"P5\n"))
        self.assertIn(b"\n3 2\n255\n", data)
        self.assertEqual(data[-6:], bytes((254, 205, 0, 0, 254, 205)))  # 그림은 윗줄(y=1)부터: 빈 곳, 애매(모름), 벽 / 벽, 빈 곳, 모름

    def test_round_trip_gives_the_same_map(self) -> None:
        loaded = load_map(save_map(grid(), self.dir / "room"), frame_id="map", stamp=5.0)
        self.assertEqual(loaded.data, (OCCUPIED, FREE, UNKNOWN, FREE, UNKNOWN, OCCUPIED))  # 애매한 50은 모름이 된다 (trinary)
        self.assertEqual((loaded.info.width, loaded.info.height, loaded.info.resolution), (3, 2, 0.05))
        o = loaded.info.origin
        self.assertEqual((o.position.x, o.position.y), (1.0, -2.0))
        self.assertAlmostEqual(2 * math.atan2(o.orientation.z, o.orientation.w), YAW)
        self.assertEqual((loaded.header.frame_id, loaded.header.stamp), ("map", 5.0))

    def test_reads_maps_written_by_ros(self) -> None:
        (self.dir / "ros.pgm").write_bytes(b"P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n2 1\n255\n" + bytes((0, 254)))
        (self.dir / "ros.yaml").write_text("image: ros.pgm\nresolution: 0.050000\norigin: [-10.0, -10.0, 0.000000]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196  # 주석\n\n")
        self.assertEqual(load_map(self.dir / "ros.yaml").data, (OCCUPIED, FREE))

    def test_missing_or_broken_files(self) -> None:
        with self.assertRaises(FileNotFoundError):
            load_map(self.dir / "none.yaml")
        (self.dir / "bad.yaml").write_text("image: bad.pgm\nresolution: 0.05\n")  # origin이 없다
        with self.assertRaises(MapIoError):
            load_map(self.dir / "bad.yaml")
        with self.assertRaises(MapIoError):
            save_map(OccupancyGrid(Header(0.0), MapMetaData(0.0, 0.05, 2, 2), (0,)), self.dir / "x")  # 칸 수가 모자란다


class TestMapNodes(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.dir = Path(directory.name)
        save_map(grid(), self.dir / "start")
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            folder = {"maps_directory": str(self.dir)}
            made["user"] = Node(ctx, "User")
            return [made["user"], MapServer(ctx, parameters={**folder, "yaml_filename": "start.yaml"}), MapSaver(ctx, parameters={**folder, "save_map_timeout": 0.2})]

        self.kros = Kros(create, time_source=SimulatedTimeSource())
        self.addCleanup(self.kros.shutdown)
        self.user = made["user"]
        self.maps: list[OccupancyGrid] = []
        self.user.subscribe("map", OccupancyGrid, self.maps.append, QOS_LATCHED)
        self.kros.context.topic_manager.join()

    def test_map_server_publishes_the_start_map_latched(self) -> None:
        self.assertEqual(len(self.maps), 1)  # 노드가 먼저 냈어도 latch라 받는다
        self.assertEqual(self.maps[0].header.frame_id, "map")
        self.assertEqual(self.maps[0].data[0], OCCUPIED)

    def test_save_then_load_through_services(self) -> None:
        self.assertTrue(self.user.call_service("map_saver/save_map", SaveMap, SaveMapRequest(map_url="saved"), timeout=2.0).result)
        self.assertTrue((self.dir / "saved.yaml").exists() and (self.dir / "saved.pgm").exists())
        response = self.user.call_service("map_server/load_map", LoadMap, LoadMapRequest(str(self.dir / "saved.yaml")), timeout=2.0)
        self.assertEqual(response.result, LoadMapResponse.RESULT_SUCCESS)
        self.kros.context.topic_manager.join()
        self.assertEqual(len(self.maps), 2)
        self.assertEqual(self.maps[-1].data, response.map.data)

    def test_load_failures_keep_the_current_map(self) -> None:
        with self.assertLogs("kros.node.map_server", level="ERROR"):
            response = self.user.call_service("map_server/load_map", LoadMap, LoadMapRequest("nothing.yaml"), timeout=2.0)
        self.assertEqual(response.result, LoadMapResponse.RESULT_MAP_DOES_NOT_EXIST)
        (self.dir / "broken.yaml").write_text("image: broken.pgm\n")
        with self.assertLogs("kros.node.map_server", level="ERROR"):
            response = self.user.call_service("map_server/load_map", LoadMap, LoadMapRequest("broken.yaml"), timeout=2.0)
        self.assertEqual(response.result, LoadMapResponse.RESULT_INVALID_MAP_DATA)
        self.kros.context.topic_manager.join()
        self.assertEqual(len(self.maps), 1)  # 실패하면 새 지도를 내지 않는다

    def test_save_fails_when_no_map_arrives(self) -> None:
        with self.assertLogs("kros.node.map_saver", level="ERROR"):
            response = self.user.call_service("map_saver/save_map", SaveMap, SaveMapRequest(map_topic="no_map", map_url="x"), timeout=5.0)
        self.assertFalse(response.result)
        self.assertFalse((self.dir / "x.yaml").exists())


if __name__ == "__main__":
    unittest.main()
