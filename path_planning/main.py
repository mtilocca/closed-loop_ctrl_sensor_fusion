"""
main.py — CLI entry point for the A* + Reeds-Shepp + Clothoid path planner.

Outputs a waypoint JSON scenario consumable by the Go runner (waypoint_pid mode).

Usage:
    # Free-space RS path:
    python3 path_planner.py --start "0,0,0" --goal "50,30,90"

    # Clothoid arcs (G2-continuous steering):
    python3 path_planner.py --start "0,0,0" --goal "50,30,90" --clothoid

    # Via-point chaining (forced cusp, replicates full_cruise_maneuver):
    python3 path_planner.py --start "0,0,0" --via "80,30,45" --goal "120,0,0" --clothoid

    # Forward-only (Dubins-equivalent, no reversing):
    python3 path_planner.py --start "0,0,0" --goal "50,30,90" --no-reverse

    # With obstacle map (white=free, black=obstacle):
    python3 path_planner.py --start "0,0,0" --goal "50,30,90" \\
        --obstacles map.png --scale 1.0 --output path.json

Coordinate convention:
    x = East (m), y = North (m), yaw = CCW from East (degrees).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from .helpers import VehicleConfig, parse_pose, wrap
from .reeds_shepp import RSPath, ReedsSheppPlanner
from .astar import AStarPlanner


# ---------------------------------------------------------------------------
# PathPlanner — top-level orchestrator
# ---------------------------------------------------------------------------

class PathPlanner:
    """
    High-level path planning orchestrator.

    Chains RS legs via optional via-points, applies clothoid arc smoothing,
    and produces a Go runner–compatible JSON scenario.

    Args:
        vehicle: VehicleConfig instance (kinematic parameters).

    Example::

        vehicle = VehicleConfig(wheelbase_m=5.5, max_steer_deg=15.0)
        planner = PathPlanner(vehicle)
        scenario = planner.plan(
            start=(0, 0, 0),
            goal=(60, 40, math.pi/2),
            clothoid=True,
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
        start:        Tuple[float, float, float],   # (x_m, y_m, yaw_rad)
        goal:         Tuple[float, float, float],
        via:          Optional[List[Tuple[float, float, float]]] = None,
        clothoid:     bool  = False,
        no_reverse:   bool  = False,
        obstacle_map: Optional[np.ndarray] = None,
        grid_res:     float = 1.0,
        step:         float = 2.0,
        sample_step:  float = 0.5,
    ) -> dict:
        """
        Plan a path from start to goal (optionally via intermediate poses).

        Args:
            start:        (x_m, y_m, yaw_rad) start pose.
            goal:         (x_m, y_m, yaw_rad) goal pose.
            via:          Optional list of intermediate (x_m, y_m, yaw_rad) poses.
                          A forced zero-speed waypoint is inserted at each junction.
            clothoid:     Use full clothoid arcs (G2-continuous curvature).
            no_reverse:   Forward-only mode (Dubins-equivalent).
            obstacle_map: 2-D bool occupancy grid (True = obstacle).
            grid_res:     A* grid resolution in metres.
            step:         Waypoint spacing in the output JSON (metres).
            sample_step:  Path sampling interval for internal poses (metres).

        Returns:
            Scenario dict ready for json.dumps().

        Raises:
            RuntimeError: If no valid path exists for any leg.
        """
        chain: List[Tuple[float, float, float]] = [start] + (via or []) + [goal]
        curve_mode = "clothoid" if clothoid else "circular arc"
        print(f"  Planning {len(chain)-1} leg(s)  curve={curve_mode}  "
              f"no_reverse={no_reverse}")

        all_waypoints:   List[dict]                            = []
        all_poses_world: List[Tuple[float, float, float, int]] = []
        last_poses:      List[Tuple[float, float, float, int]] = []

        for leg_idx in range(len(chain) - 1):
            cx0, cy0, cyaw0 = chain[leg_idx]
            cx1, cy1, cyaw1 = chain[leg_idx + 1]
            is_last_leg     = (leg_idx == len(chain) - 2)

            # Plan leg
            leg_path: Optional[RSPath] = None
            if obstacle_map is None:
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
                    print("  WARNING: --no-reverse not applied inside A* (obstacle mode).")
                leg_path = self.astar.plan(
                    (cx0, cy0, cyaw0), (cx1, cy1, cyaw1),
                    grid_res=grid_res,
                    obstacle_map=obstacle_map,
                )
                if leg_path is not None:
                    print(f"  Leg {leg_idx+1}: A* {leg_path.total_length:.1f} m")

            if leg_path is None:
                raise RuntimeError(
                    f"No valid path for leg {leg_idx+1} "
                    f"({cx0:.1f},{cy0:.1f},{math.degrees(cyaw0):.0f}°) → "
                    f"({cx1:.1f},{cy1:.1f},{math.degrees(cyaw1):.0f}°)."
                )

            # Sample leg
            r      = self.vehicle.r_min
            poses  = leg_path.sample(r, step=sample_step, clothoid=clothoid)
            last_poses = poses

            # Accumulate world-frame poses for plotting
            for lx, ly, lyaw, d in poses:
                c_r = math.cos(cyaw0); s_r = math.sin(cyaw0)
                wx  = cx0 + c_r * lx - s_r * ly
                wy  = cy0 + s_r * lx + c_r * ly
                all_poses_world.append((wx, wy, wrap(cyaw0 + lyaw), d))

            # Extract waypoints for this leg
            wps = self.rs.path_to_waypoints(poses, cx0, cy0, cyaw0,
                                             sample_step=step)
            if not is_last_leg and wps:
                wps[-1]["comment"]          = f"via-point {leg_idx+1}: forced stop"
                wps[-1]["target_speed_mps"] = 0.0

            all_waypoints.extend(wps)

        print(f"  Generated {len(all_waypoints)} waypoints total (step={step} m)")

        sx, sy, syaw_rad = start
        gx, gy, gyaw_rad = goal
        scenario = self.rs.build_scenario_json(
            all_waypoints,
            (sx, sy, math.degrees(syaw_rad)),
            (gx, gy, math.degrees(gyaw_rad)),
        )
        # Stash for downstream use (e.g. plot)
        self._last_poses_world = all_poses_world
        self._last_poses       = last_poses
        self._last_waypoints   = all_waypoints

        return scenario

    # ------------------------------------------------------------------
    # Convenience plotting
    # ------------------------------------------------------------------

    def plot_last(self) -> None:
        """Plot the result of the most recent plan() call."""
        if not hasattr(self, "_last_poses"):
            print("No path planned yet — call plan() first.")
            return
        sx, sy, syaw_rad = getattr(self, "_start", (0.0, 0.0, 0.0))
        gx, gy, gyaw_rad = getattr(self, "_goal",  (0.0, 0.0, 0.0))
        self.rs.plot(
            self._last_poses, sx, sy, syaw_rad, gx, gy, gyaw_rad,
            self._last_waypoints,
            poses_world=self._last_poses_world if self._last_poses_world else None,
        )


# ---------------------------------------------------------------------------
# CLI argument parser — handles negative-number coordinates
# ---------------------------------------------------------------------------

class _PoseParser(argparse.ArgumentParser):
    """ArgumentParser that treats 'x,y,yaw' strings with negative x as values,
    not as unknown flags (e.g. '--goal=-20,0,0' or '--goal -20,0,0')."""

    def _parse_optional(self, arg_string: str):
        if arg_string.startswith("-") and "," in arg_string:
            try:
                float(arg_string.split(",")[0])
                return None   # signals "not an option string"
            except ValueError:
                pass
        return super()._parse_optional(arg_string)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = _PoseParser(
        description="A* + Reeds-Shepp + Clothoid path planner for XCMG XDE360",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--start", required=True, metavar="X,Y,YAW",
                        help="Start pose: 'x_m,y_m,yaw_deg'")
    parser.add_argument("--goal", required=True, metavar="X,Y,YAW",
                        help="Goal pose: 'x_m,y_m,yaw_deg'")
    parser.add_argument("--via", action="append", metavar="X,Y,YAW",
                        help="Forced stop waypoint (repeatable). Chains RS segments. "
                             "Example: --via '80,30,45' --via '100,10,0'")
    parser.add_argument("--output", default=None,
                        help="Output JSON file path (omit to print to stdout)")
    parser.add_argument("--plot", action="store_true",
                        help="Show matplotlib visualisation after planning")
    parser.add_argument("--clothoid", action="store_true",
                        help="Replace circular arcs with full clothoid arcs "
                             "(G2-continuous steering). Arc segments become 2× longer.")
    parser.add_argument("--no-reverse", action="store_true",
                        help="Disable reverse segments (forward-only, Dubins-equivalent). "
                             "Incompatible with A* obstacle mode.")
    parser.add_argument("--obstacles", default=None,
                        help="PNG obstacle map (white=free, black=obstacle)")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Obstacle map scale: metres per pixel (default 1.0)")
    parser.add_argument("--step", type=float, default=2.0,
                        help="Waypoint spacing in metres (default 2.0)")
    parser.add_argument("--grid-res", type=float, default=1.0,
                        help="A* grid resolution in metres (default 1.0)")

    # Vehicle parameters
    _defaults = VehicleConfig()
    parser.add_argument("--wheelbase",     type=float, default=_defaults.wheelbase_m,
                        help=f"Vehicle wheelbase in m (default {_defaults.wheelbase_m})")
    parser.add_argument("--max-steer",     type=float, default=_defaults.max_steer_deg,
                        help=f"Max steering angle in deg (default {_defaults.max_steer_deg})")
    parser.add_argument("--fwd-speed",     type=float, default=_defaults.cruise_fwd_mps,
                        help=f"Forward cruise speed m/s (default {_defaults.cruise_fwd_mps})")
    parser.add_argument("--rev-speed",     type=float, default=_defaults.cruise_rev_mps,
                        help=f"Reverse cruise speed m/s (default {_defaults.cruise_rev_mps})")
    parser.add_argument("--arrive-radius", type=float, default=_defaults.arrive_radius_m,
                        help=f"Waypoint arrival radius in m (default {_defaults.arrive_radius_m})")

    args = parser.parse_args()

    # Build VehicleConfig from CLI args
    vehicle = VehicleConfig(
        wheelbase_m     = args.wheelbase,
        max_steer_deg   = args.max_steer,
        cruise_fwd_mps  = args.fwd_speed,
        cruise_rev_mps  = args.rev_speed,
        arrive_radius_m = args.arrive_radius,
    )
    print(f"  {vehicle}")
    print(f"  Minimum turning radius: {vehicle.r_min:.2f} m")

    # Parse poses
    sx, sy, syaw_deg = parse_pose(args.start)
    gx, gy, gyaw_deg = parse_pose(args.goal)
    syaw_rad = math.radians(syaw_deg)
    gyaw_rad = math.radians(gyaw_deg)
    print(f"  Start: ({sx}, {sy}, {syaw_deg}°)")
    print(f"  Goal:  ({gx}, {gy}, {gyaw_deg}°)")

    via_world: List[Tuple[float, float, float]] = []
    for v_str in (args.via or []):
        vx, vy, vyaw_deg = parse_pose(v_str)
        via_world.append((vx, vy, math.radians(vyaw_deg)))
        print(f"  Via:   ({vx}, {vy}, {vyaw_deg}°)")

    # Load obstacle map
    obstacle_map: Optional[np.ndarray] = None
    if args.obstacles:
        try:
            from PIL import Image
            img          = np.array(Image.open(args.obstacles).convert("L"))
            obstacle_map = img < 128
            print(f"  Obstacle map: {obstacle_map.shape} @ {args.scale} m/px")
        except ImportError:
            print("  WARNING: Pillow not installed — ignoring obstacle map.")

    # Run planner
    planner = PathPlanner(vehicle)
    try:
        scenario = planner.plan(
            start        = (sx, sy, syaw_rad),
            goal         = (gx, gy, gyaw_rad),
            via          = via_world or None,
            clothoid     = args.clothoid,
            no_reverse   = args.no_reverse,
            obstacle_map = obstacle_map,
            grid_res     = args.grid_res,
            step         = args.step,
        )
    except RuntimeError as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

    # Output
    json_str = json.dumps(scenario, indent=2)
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json_str)
        print(f"  Saved: {args.output}")
    else:
        print(json_str)

    # Plot
    if args.plot:
        planner._start = (sx, sy, syaw_rad)
        planner._goal  = (gx, gy, gyaw_rad)
        planner.plot_last()

    print("Done.")
