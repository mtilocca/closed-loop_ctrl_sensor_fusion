"""
astar.py — A* search on the SE(2) state lattice using Reeds-Shepp motion
           primitives.

For free-space planning use ReedsSheppPlanner.plan() directly (faster).
AStarPlanner is used when an obstacle occupancy map is provided.

Usage::

    vehicle = VehicleConfig()
    planner = AStarPlanner(vehicle)
    path    = planner.plan(
        start=(0, 0, 0),
        goal=(50, 30, math.pi/2),
        obstacle_map=occ_grid,
    )
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .geometry     import Geometry
from .helpers      import VehicleConfig
from .rs_path      import Segment, RSPath
from .reeds_shepp  import ReedsSheppPlanner


@dataclass(order=True)
class _Node:
    """A* search node on the SE(2) lattice."""
    f:      float
    g:      float               = field(compare=False)
    state:  Tuple[int, int, int] = field(compare=False)
    path:   Optional[RSPath]    = field(compare=False, default=None)
    parent: Optional["_Node"]   = field(compare=False, default=None)


class AStarPlanner:
    """
    A* path search on the SE(2) lattice using Reeds-Shepp motion primitives.

    Each lattice node is a discretised (x_grid, y_grid, yaw_bin) state.
    Edges are RS motion primitives; collision is checked against an optional
    occupancy grid.  When no obstacle map is given, the first RS path to the
    goal is returned directly (same as ReedsSheppPlanner.plan()).

    Args:
        vehicle: VehicleConfig instance.

    Example::

        vehicle = VehicleConfig()
        planner = AStarPlanner(vehicle)
        path    = planner.plan(
            start=(0, 0, 0),
            goal=(50, 30, math.pi / 2),
            obstacle_map=occ_grid,
        )
    """

    def __init__(self, vehicle: VehicleConfig) -> None:
        self.vehicle = vehicle
        self._rs     = ReedsSheppPlanner(vehicle)

    def plan(
        self,
        start:        Tuple[float, float, float],
        goal:         Tuple[float, float, float],
        grid_res:     float = 1.0,
        yaw_bins:     int   = 36,
        obstacle_map: Optional[np.ndarray] = None,
    ) -> Optional[RSPath]:
        """
        Plan a collision-free path using A* on the SE(2) lattice.

        Args:
            start:        (x_m, y_m, yaw_rad) start pose.
            goal:         (x_m, y_m, yaw_rad) goal pose.
            grid_res:     Lattice cell size in metres (default 1.0).
            yaw_bins:     Heading discretisation bins (default 36 = 10° each).
            obstacle_map: 2-D bool array (True = obstacle), or None for free space.

        Returns:
            RSPath from start to goal, or None if no collision-free path exists.
        """
        sx, sy, syaw = start
        gx, gy, gyaw = goal

        def _disc(x: float, y: float, yaw: float) -> Tuple[int, int, int]:
            xi = round(x / grid_res)
            yi = round(y / grid_res)
            yb = round(((yaw % (2 * math.pi)) / (2 * math.pi)) * yaw_bins) % yaw_bins
            return xi, yi, yb

        def _world(xi: int, yi: int, yb: int) -> Tuple[float, float, float]:
            return xi * grid_res, yi * grid_res, (yb / yaw_bins) * 2 * math.pi

        gs = _disc(sx, sy, syaw)
        gg = _disc(gx, gy, gyaw)

        h0         = math.hypot((gs[0] - gg[0]) * grid_res,
                                (gs[1] - gg[1]) * grid_res)
        start_node = _Node(f=h0, g=0.0, state=gs)

        open_heap: List[_Node] = [start_node]
        visited:   dict        = {}

        while open_heap:
            node = heapq.heappop(open_heap)
            s    = node.state
            if s in visited:
                continue
            visited[s] = node

            wx, wy, wyaw = _world(*s)

            # Try direct RS connection to goal
            direct = self._rs.plan(wx, wy, wyaw, gx, gy, gyaw)
            if direct is not None and not self._collision(
                    direct, wx, wy, wyaw, obstacle_map, grid_res):
                segs: List[Segment] = []
                cur = node
                while cur.parent is not None:
                    segs = list(cur.path.segments) + segs
                    cur  = cur.parent
                segs += direct.segments
                return RSPath(segs, sum(sg.length for sg in segs))

            # Expand neighbours
            for dangle in [0.0, math.pi/4, math.pi/2, math.pi,
                           -math.pi/4, -math.pi/2]:
                nb_yaw = Geometry.wrap(wyaw + dangle)
                ns = _disc(wx + grid_res * math.cos(nb_yaw),
                           wy + grid_res * math.sin(nb_yaw),
                           nb_yaw)
                if ns in visited:
                    continue
                nx, ny, nyaw = _world(*ns)
                prim = self._rs.plan(wx, wy, wyaw, nx, ny, nyaw)
                if prim is None:
                    continue
                if self._collision(prim, wx, wy, wyaw, obstacle_map, grid_res):
                    continue
                g_new = node.g + prim.total_length
                h_new = math.hypot((ns[0] - gg[0]) * grid_res,
                                   (ns[1] - gg[1]) * grid_res)
                heapq.heappush(open_heap, _Node(
                    f=g_new + h_new, g=g_new,
                    state=ns, path=prim, parent=node,
                ))

        return None

    def _collision(
        self,
        path:         RSPath,
        sx:           float,
        sy:           float,
        syaw:         float,
        obstacle_map: Optional[np.ndarray],
        grid_res:     float,
        step:         float = 0.5,
    ) -> bool:
        """Return True if path passes through any obstacle cell."""
        if obstacle_map is None:
            return False
        h, w = obstacle_map.shape
        for x, y, _, _ in path.sample(self.vehicle.r_min, step):
            wx = sx + x * math.cos(syaw) - y * math.sin(syaw)
            wy = sy + x * math.sin(syaw) + y * math.cos(syaw)
            xi, yi = int(wx / grid_res), int(wy / grid_res)
            if 0 <= xi < w and 0 <= yi < h:
                if obstacle_map[yi, xi]:
                    return True
            else:
                return True   # outside map = obstacle
        return False
