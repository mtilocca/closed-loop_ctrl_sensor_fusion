"""
astar.py — A* search on the SE(2) state lattice using Reeds-Shepp motion primitives.

For free-space planning use ReedsSheppPlanner.plan() directly (faster).
Use AStarPlanner when an obstacle map is provided.

Example::

    vehicle = VehicleConfig()
    planner = AStarPlanner(vehicle)
    path    = planner.plan((0,0,0), (50,30,math.pi/2), obstacle_map=occ_grid)
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .helpers import VehicleConfig, wrap
from .reeds_shepp import RSPath, Segment, ReedsSheppPlanner


# ---------------------------------------------------------------------------
# A* node
# ---------------------------------------------------------------------------

@dataclass(order=True)
class _Node:
    f:      float
    g:      float                       = field(compare=False)
    state:  Tuple[int, int, int]        = field(compare=False)
    path:   Optional[RSPath]            = field(compare=False, default=None)
    parent: Optional["_Node"]           = field(compare=False, default=None)


# ---------------------------------------------------------------------------
# AStarPlanner
# ---------------------------------------------------------------------------

class AStarPlanner:
    """
    A* path search on the SE(2) lattice using Reeds-Shepp motion primitives.

    Args:
        vehicle: VehicleConfig instance.

    Example::

        vehicle = VehicleConfig()
        planner = AStarPlanner(vehicle)
        path    = planner.plan(
            start=(0, 0, 0),
            goal=(50, 30, math.pi/2),
            obstacle_map=occ_grid,
        )
    """

    def __init__(self, vehicle: VehicleConfig) -> None:
        self.vehicle = vehicle
        self._rs     = ReedsSheppPlanner(vehicle)

    # ------------------------------------------------------------------
    # Collision checking
    # ------------------------------------------------------------------

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
        """Return True if path passes through an obstacle cell."""
        if obstacle_map is None:
            return False
        h, w = obstacle_map.shape
        r = self.vehicle.r_min
        for x, y, _, _ in path.sample(r, step):
            gx_w = sx + x * math.cos(syaw) - y * math.sin(syaw)
            gy_w = sy + x * math.sin(syaw) + y * math.cos(syaw)
            xi = int(gx_w / grid_res)
            yi = int(gy_w / grid_res)
            if 0 <= xi < w and 0 <= yi < h:
                if obstacle_map[yi, xi]:
                    return True
            else:
                return True   # out of map bounds = obstacle
        return False

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan(
        self,
        start:        Tuple[float, float, float],
        goal:         Tuple[float, float, float],
        grid_res:     float = 1.0,
        yaw_bins:     int   = 36,
        obstacle_map: Optional[np.ndarray] = None,
    ) -> Optional[RSPath]:
        """
        A* on SE(2) lattice using Reeds-Shepp motion primitives.

        Args:
            start:        (x_m, y_m, yaw_rad) start pose.
            goal:         (x_m, y_m, yaw_rad) goal pose.
            grid_res:     Grid cell size in metres (default 1.0).
            yaw_bins:     Heading discretisation bins (default 36 = 10° each).
            obstacle_map: 2-D bool array (True = obstacle), or None for free space.

        Returns:
            RSPath from start to goal, or None.
        """
        sx, sy, syaw = start
        gx, gy, gyaw = goal

        def _discretise(x: float, y: float, yaw: float) -> Tuple[int, int, int]:
            xi = round(x / grid_res)
            yi = round(y / grid_res)
            yb = round(((yaw % (2 * math.pi)) / (2 * math.pi)) * yaw_bins) % yaw_bins
            return xi, yi, yb

        def _world(xi: int, yi: int, yawi: int) -> Tuple[float, float, float]:
            return (xi * grid_res,
                    yi * grid_res,
                    (yawi / yaw_bins) * 2 * math.pi)

        def _heuristic(xi, yi, gxi, gyi):
            dx = (xi - gxi) * grid_res
            dy = (yi - gyi) * grid_res
            return math.hypot(dx, dy)

        gs_state = _discretise(sx, sy, syaw)
        gg_state = _discretise(gx, gy, gyaw)

        h0         = _heuristic(*gs_state[:2], *gg_state[:2])
        start_node = _Node(f=h0, g=0.0, state=gs_state)

        open_heap: List[_Node] = [start_node]
        visited:   dict        = {}

        while open_heap:
            node = heapq.heappop(open_heap)
            s    = node.state
            if s in visited:
                continue
            visited[s] = node

            # Try direct RS path to goal from current node
            wx, wy, wyaw = _world(*s)
            direct = self._rs.plan(wx, wy, wyaw, gx, gy, gyaw)
            if (direct is not None
                    and not self._collision(direct, wx, wy, wyaw,
                                            obstacle_map, grid_res)):
                # Reconstruct full path segments back to start
                path_segs: List[Segment] = []
                cur = node
                while cur.parent is not None:
                    path_segs = list(cur.path.segments) + path_segs
                    cur = cur.parent
                path_segs += direct.segments
                total = sum(sg.length for sg in path_segs)
                return RSPath(path_segs, total)

            # Expand neighbours via RS motion primitives
            for dangle in [0.0, math.pi/4, math.pi/2, math.pi,
                           -math.pi/4, -math.pi/2]:
                nb_yaw = wrap(wyaw + dangle)
                nxi, nyi, nyawi = _discretise(
                    wx + grid_res * math.cos(nb_yaw),
                    wy + grid_res * math.sin(nb_yaw),
                    nb_yaw,
                )
                if (nxi, nyi, nyawi) in visited:
                    continue
                nx, ny, nyaw = _world(nxi, nyi, nyawi)
                prim = self._rs.plan(wx, wy, wyaw, nx, ny, nyaw)
                if prim is None:
                    continue
                if self._collision(prim, wx, wy, wyaw, obstacle_map, grid_res):
                    continue
                g_new = node.g + prim.total_length
                h_new = _heuristic(nxi, nyi, *gg_state[:2])
                child = _Node(
                    f=g_new + h_new, g=g_new,
                    state=(nxi, nyi, nyawi),
                    path=prim, parent=node,
                )
                heapq.heappush(open_heap, child)

        return None


# ---------------------------------------------------------------------------
# Back-compat module-level function
# ---------------------------------------------------------------------------

def astar(
    start:        Tuple[float, float, float],
    goal:         Tuple[float, float, float],
    grid_res:     float = 1.0,
    yaw_bins:     int   = 36,
    obstacle_map: Optional[np.ndarray] = None,
    _vehicle:     Optional[VehicleConfig] = None,
) -> Optional[RSPath]:
    """Module-level wrapper for AStarPlanner.plan (back-compat)."""
    from .helpers import VehicleConfig as _VC
    v = _vehicle or _VC()
    return AStarPlanner(v).plan(start, goal, grid_res, yaw_bins, obstacle_map)
