"""
path_planner.py — High-level path planning orchestrator and CLI argument parser.

PathPlanner chains multiple RS/A* legs via optional via-points, applies
clothoid arc smoothing, snaps the final waypoint to the exact goal, and
produces a Go runner–compatible JSON scenario.

Rectangular obstacles can be specified as (x_min, y_min, x_max, y_max) tuples.
When present, A* is used automatically to route around them.

_PoseParser is a thin argparse.ArgumentParser subclass that handles negative
coordinate values such as '--goal -20,0,0' without misidentifying them as flags.
"""

from __future__ import annotations

import argparse
import math
from typing import List, Optional, Tuple

import numpy as np

from .geometry    import Geometry
from .helpers     import VehicleConfig
from .rs_path     import RSPath
from .reeds_shepp import ReedsSheppPlanner
from .astar       import AStarPlanner


class _PoseParser(argparse.ArgumentParser):
    """
    ArgumentParser that treats 'x,y,yaw' strings with a negative leading
    number as positional values, not as unknown flags.

    Without this override, '--goal -20,0,0' raises::

        error: argument --goal: expected one argument

    because argparse sees the leading '-' and interprets the string as a flag.
    """

    def _parse_optional(self, arg_string: str):
        if arg_string.startswith("-") and "," in arg_string:
            try:
                float(arg_string.split(",")[0])
                return None   # treat as a positional value, not a flag
            except ValueError:
                pass
        return super()._parse_optional(arg_string)


class PathPlanner:
    """
    High-level orchestrator: chains RS/A* legs via optional via-points,
    applies clothoid arc smoothing, snaps the final waypoint to the exact
    goal, and produces a Go runner JSON scenario.

    Rectangular obstacles are built into an occupancy grid and routed around
    using AStarPlanner automatically.

    Optionally refines the trajectory with CasADiOptimizer for minimum-jerk,
    kinematically smooth waypoints.

    Args:
        vehicle: VehicleConfig instance (kinematic parameters).

    Example::

        vehicle = VehicleConfig()
        planner = PathPlanner(vehicle)
        scenario = planner.plan(
            start=(0, 0, 0),
            goal=(100, 50, math.pi/2),
            clothoid=True,
            rect_obstacles=[(20, 10, 40, 35), (60, 20, 80, 50)],
        )
    """

    def __init__(self, vehicle: VehicleConfig) -> None:
        self.vehicle = vehicle
        self.rs      = ReedsSheppPlanner(vehicle)
        self.astar   = AStarPlanner(vehicle)

    # ------------------------------------------------------------------
    # Main planning method
    # ------------------------------------------------------------------

    def plan(
        self,
        start:          Tuple[float, float, float],
        goal:           Tuple[float, float, float],
        via:            Optional[List[Tuple[float, float, float]]] = None,
        clothoid:       bool  = False,
        no_reverse:     bool  = False,
        casadi:         bool  = False,
        casadi_dt:      float = 0.2,
        rect_obstacles: Optional[List[Tuple[float, float, float, float]]] = None,
        obstacle_map:   Optional[np.ndarray] = None,
        obs_x_offset:   float = 0.0,
        obs_y_offset:   float = 0.0,
        grid_res:       float = 1.0,
        step:           float = 2.0,
        sample_step:    float = 0.5,
    ) -> dict:
        """
        Plan a path from start to goal, optionally via intermediate poses.

        Args:
            start:          (x_m, y_m, yaw_rad) start pose.
            goal:           (x_m, y_m, yaw_rad) goal pose.
            via:            Intermediate (x_m, y_m, yaw_rad) poses.
                            A forced zero-speed waypoint is inserted at each.
            clothoid:       Replace circular arcs with full clothoid (G2-smooth).
                            The final waypoint is snapped to the exact goal
                            position to compensate for clothoid endpoint drift.
            no_reverse:     Forward-only mode (Dubins-equivalent).
            casadi:         Refine result with CasADi/IPOPT trajectory optimiser.
            casadi_dt:      Integration timestep for CasADi (seconds).
            rect_obstacles: List of (x_min, y_min, x_max, y_max) world-frame
                            obstacle rectangles.  Forces A* routing.
            obstacle_map:   Pre-built 2-D bool occupancy grid (True = obstacle).
                            Ignored if rect_obstacles is provided.
            obs_x_offset:   X origin of the obstacle_map grid in world coords.
            obs_y_offset:   Y origin of the obstacle_map grid in world coords.
            grid_res:       A* grid resolution in metres.
            step:           Waypoint spacing in output JSON (metres).
            sample_step:    Path sampling interval for internal poses (metres).

        Returns:
            Scenario dict ready for json.dumps().

        Raises:
            RuntimeError: If no valid path exists for any leg, or CasADi fails.
        """
        # Build obstacle grid from rectangles if provided
        if rect_obstacles:
            obstacle_map, obs_x_offset, obs_y_offset = self._build_obstacle_grid(
                rect_obstacles, start, goal, via or [], grid_res,
            )

        chain = [start] + (via or []) + [goal]
        curve = "clothoid" if clothoid else "circular arc"
        using_astar = obstacle_map is not None
        print(f"  Planning {len(chain)-1} leg(s)  curve={curve}  "
              f"router={'A*' if using_astar else 'RS'}  "
              f"no_reverse={no_reverse}  casadi={casadi}")

        all_waypoints:   List[dict]                             = []
        all_poses_world: List[Tuple[float, float, float, int]] = []
        last_poses:      List[Tuple[float, float, float, int]] = []

        for leg_idx in range(len(chain) - 1):
            cx0, cy0, cyaw0 = chain[leg_idx]
            cx1, cy1, cyaw1 = chain[leg_idx + 1]
            is_last         = leg_idx == len(chain) - 2

            leg_path: Optional[RSPath] = None
            if not using_astar:
                leg_path = self.rs.plan(cx0, cy0, cyaw0, cx1, cy1, cyaw1,
                                        no_reverse=no_reverse)
                if leg_path is not None:
                    cusps = sum(
                        1 for i in range(1, len(leg_path.segments))
                        if leg_path.segments[i].dir != leg_path.segments[i-1].dir
                    )
                    print(f"  Leg {leg_idx+1}: RS {leg_path.total_length:.1f} m, "
                          f"{len(leg_path.segments)} segs, {cusps} cusps")
            else:
                if no_reverse:
                    print("  WARNING: --no-reverse ignored in A* obstacle mode.")
                leg_path = self.astar.plan(
                    (cx0, cy0, cyaw0), (cx1, cy1, cyaw1),
                    grid_res=grid_res,
                    obstacle_map=obstacle_map,
                    obs_x_offset=obs_x_offset,
                    obs_y_offset=obs_y_offset,
                )
                if leg_path is not None:
                    print(f"  Leg {leg_idx+1}: A* {leg_path.total_length:.1f} m")

            if leg_path is None:
                raise RuntimeError(
                    f"No valid path for leg {leg_idx+1} "
                    f"({cx0:.1f},{cy0:.1f},{math.degrees(cyaw0):.0f}°) → "
                    f"({cx1:.1f},{cy1:.1f},{math.degrees(cyaw1):.0f}°)."
                )

            poses      = leg_path.sample(self.vehicle.r_min,
                                         step=sample_step, clothoid=clothoid)
            last_poses = poses

            for lx, ly, lyaw, d in poses:
                c_r = math.cos(cyaw0); s_r = math.sin(cyaw0)
                all_poses_world.append((
                    cx0 + c_r * lx - s_r * ly,
                    cy0 + s_r * lx + c_r * ly,
                    Geometry.wrap(cyaw0 + lyaw), d,
                ))

            wps = self.rs.path_to_waypoints(poses, cx0, cy0, cyaw0,
                                            sample_step=step)
            if not is_last and wps:
                wps[-1]["comment"]          = f"via-point {leg_idx+1}: forced stop"
                wps[-1]["target_speed_mps"] = 0.0

            all_waypoints.extend(wps)

        # Snap the final waypoint to the exact goal to correct clothoid drift
        if all_waypoints:
            gx, gy, gyaw_rad = goal
            all_waypoints[-1]["x_m"]     = round(gx, 3)
            all_waypoints[-1]["y_m"]     = round(gy, 3)
            all_waypoints[-1]["yaw_deg"] = round(math.degrees(gyaw_rad), 2)

        print(f"  Generated {len(all_waypoints)} waypoints total (step={step} m)")

        # Optional CasADi refinement
        if casadi:
            from .casadi_optimizer import CasADiOptimizer
            print("  Running CasADi/IPOPT trajectory optimisation…")
            optimizer     = CasADiOptimizer(self.vehicle)
            all_waypoints = optimizer.optimize(all_waypoints, dt=casadi_dt)
            print(f"  CasADi refined to {len(all_waypoints)} knots")

        sx, sy, syaw_rad = start
        gx, gy, gyaw_rad = goal
        scenario = self.rs.build_scenario_json(
            all_waypoints,
            (sx, sy, math.degrees(syaw_rad)),
            (gx, gy, math.degrees(gyaw_rad)),
        )

        self._last_poses_world  = all_poses_world
        self._last_poses        = last_poses
        self._last_waypoints    = all_waypoints
        self._last_obstacles    = rect_obstacles or []

        return scenario

    # ------------------------------------------------------------------
    # Convenience plotting
    # ------------------------------------------------------------------

    def plot_last(
        self,
        start: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        goal:  Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Plot the result of the most recent plan() call."""
        if not hasattr(self, "_last_poses"):
            print("No path planned yet — call plan() first.")
            return
        sx, sy, syaw_rad = start
        gx, gy, gyaw_rad = goal
        self.rs.plot(
            self._last_poses, sx, sy, syaw_rad, gx, gy, gyaw_rad,
            self._last_waypoints,
            poses_world = self._last_poses_world or None,
            obstacles   = self._last_obstacles or None,
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _build_obstacle_grid(
        self,
        rects:    List[Tuple[float, float, float, float]],
        start:    Tuple[float, float, float],
        goal:     Tuple[float, float, float],
        via:      List[Tuple[float, float, float]],
        grid_res: float,
        padding:  float = 30.0,
    ) -> Tuple[np.ndarray, float, float]:
        """
        Build a bool occupancy grid from a list of obstacle rectangles.

        The grid is auto-sized to encompass all poses and obstacles with
        extra padding so A* can explore paths that detour around obstacles.

        Returns:
            (grid, x_offset, y_offset) where grid[row, col] is True for
            obstacle cells, and (x_offset, y_offset) is the world-coordinate
            origin of cell (0, 0).
        """
        # Gather all key X/Y values to size the map
        key_x = [start[0], goal[0]] + [p[0] for p in via]
        key_y = [start[1], goal[1]] + [p[1] for p in via]
        for x0, y0, x1, y1 in rects:
            key_x += [x0, x1]
            key_y += [y0, y1]

        pad      = max(padding, self.vehicle.r_min * 4)
        x_min    = min(key_x) - pad
        y_min    = min(key_y) - pad
        x_max    = max(key_x) + pad
        y_max    = max(key_y) + pad

        cols = int((x_max - x_min) / grid_res) + 2
        rows = int((y_max - y_min) / grid_res) + 2
        grid = np.zeros((rows, cols), dtype=bool)

        for x0, y0, x1, y1 in rects:
            c0 = max(0, int((min(x0, x1) - x_min) / grid_res))
            r0 = max(0, int((min(y0, y1) - y_min) / grid_res))
            c1 = min(cols, int((max(x0, x1) - x_min) / grid_res) + 1)
            r1 = min(rows, int((max(y0, y1) - y_min) / grid_res) + 1)
            grid[r0:r1, c0:c1] = True

        print(f"  Obstacle grid: {cols}×{rows} cells @ {grid_res} m/cell  "
              f"origin=({x_min:.0f},{y_min:.0f})  "
              f"{int(grid.sum())} obstacle cells")
        return grid, x_min, y_min
