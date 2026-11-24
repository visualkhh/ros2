"""nav2 구성요소 마이크로벤치마크 (Python kros). C++ krosc_nav2_bench와 같은 맵/횟수."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose
from kros.msgs.nav_msgs import MapMetaData, OccupancyGrid
from kros.msgs.std_msgs import Header
from kros.nodes.nav2.controller.dwb import DWBLocalPlanner
from kros.nodes.nav2.controller.mppi import MPPIController
from kros.nodes.nav2.controller.regulated_pure_pursuit import RegulatedPurePursuitController
from kros.nodes.nav2.costmap_2d import Costmap
from kros.nodes.nav2.planner.navfn_planner import NavfnPlanner
from kros.nodes.nav2.planner.smac_planner_2d import SmacPlanner2D
from kros.nodes.nav2.planner.smac_planner_hybrid import SmacPlannerHybrid
from kros.nodes.nav2.planner.theta_star_planner import ThetaStarPlanner
from kros.nodes.node import Node

SIZE = 200
RESOLUTION = 0.05


def make_map() -> OccupancyGrid:
    data = [0] * (SIZE * SIZE)

    def wall(x0: int, y0: int, x1: int, y1: int) -> None:
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                data[y * SIZE + x] = 100

    wall(0, 0, SIZE - 1, 2)
    wall(0, SIZE - 3, SIZE - 1, SIZE - 1)
    wall(0, 0, 2, SIZE - 1)
    wall(SIZE - 3, 0, SIZE - 1, SIZE - 1)
    for y in range(20, 170, 30):
        wall(20, y, 170, y + 1)
    wall(100, 20, 101, 110)
    wall(100, 140, 101, 180)
    return OccupancyGrid(
        Header(0.0, "map"),
        MapMetaData(0.0, RESOLUTION, SIZE, SIZE, Pose(Point(0.0, 0.0, 0.0))),
        tuple(data),
    )


def measure(label: str, iterations: int, function) -> None:
    start = time.perf_counter()
    for _ in range(iterations):
        function()
    total = (time.perf_counter() - start) * 1000.0
    print(f"{label:<24} {total / iterations:8.3f} ms/op  ({iterations} ops, {total:.1f} ms)")


def main() -> None:
    context = Context()
    node = Node(context, "nav2_bench")
    costmap = Costmap.from_occupancy_grid(make_map(), track_unknown_space=False)
    start = (0.5, 0.5, 0.0)
    goal = (9.5, 9.5, 0.0)

    measure(
        "costmap inflate",
        50,
        lambda: Costmap.from_occupancy_grid(make_map(), track_unknown_space=False).inflate(0.2, 0.55, 3.0),
    )

    navfn = NavfnPlanner()
    navfn.configure(node, "GridBased")
    navfn.use_astar.set(True)
    navfn.create_plan(costmap, start, goal)
    measure("Navfn (A*)", 100, lambda: navfn.create_plan(costmap, start, goal))
    points = navfn.create_plan(costmap, start, goal)
    path = [(x, y, 0.0) for x, y in points]

    theta = ThetaStarPlanner()
    theta.configure(node, "Theta")
    theta.create_plan(costmap, start, goal)
    measure("Theta*", 50, lambda: theta.create_plan(costmap, start, goal))

    smac2d = SmacPlanner2D()
    smac2d.configure(node, "Smac2D")
    smac2d.create_plan(costmap, start, goal)
    measure("Smac2D", 50, lambda: smac2d.create_plan(costmap, start, goal))

    hybrid = SmacPlannerHybrid()
    hybrid.configure(node, "SmacHybrid")
    hybrid.create_plan(costmap, start, goal)
    measure("SmacHybrid", 10, lambda: hybrid.create_plan(costmap, start, goal))

    rpp = RegulatedPurePursuitController()
    rpp.configure(node, "FollowPath")
    measure("RPP", 2000, lambda: rpp.compute_velocity_commands(start, (0.0, 0.0), path, costmap, 0.05))

    dwb = DWBLocalPlanner()
    dwb.configure(node, "DWB")
    measure("DWB", 100, lambda: dwb.compute_velocity_commands(start, (0.0, 0.0), path, costmap, 0.05))

    mppi = MPPIController()
    mppi.configure(node, "MPPI")
    measure("MPPI", 100, lambda: mppi.compute_velocity_commands(start, (0.0, 0.0), path, costmap, 0.05))


if __name__ == "__main__":
    main()
