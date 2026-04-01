"""
main.py — CLI entry point for the A* + Reeds-Shepp + Clothoid path planner.

Outputs a waypoint JSON scenario consumable by the Go runner (waypoint_pid mode).

Usage::

    # Free-space RS path:
    python3 path_planner.py --start "0,0,0" --goal "50,30,90"

    # Clothoid arcs (G2-continuous steering):
    python3 path_planner.py --start "0,0,0" --goal "50,30,90" --clothoid

    # Via-point chaining (forced cusp, replicates full_cruise_maneuver):
    python3 path_planner.py --start "0,0,0" --via "80,30,45" --goal "120,0,0" --clothoid

    # Forward-only (Dubins-equivalent, no reversing):
    python3 path_planner.py --start "0,0,0" --goal "50,30,90" --no-reverse

    # CasADi trajectory optimisation (requires pip install casadi):
    python3 path_planner.py --start "0,0,0" --goal "50,30,90" --clothoid --casadi

    # With obstacle map (white=free, black=obstacle):
    python3 path_planner.py --start "0,0,0" --goal "50,30,90" \\
        --obstacles map.png --scale 1.0 --output path.json

Coordinate convention:
    x = East (m), y = North (m), yaw = CCW from East (degrees).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from .geometry     import Geometry
from .helpers      import VehicleConfig
from .path_planner import PathPlanner, _PoseParser


def main() -> None:
    parser = _PoseParser(
        description="A* + Reeds-Shepp + Clothoid path planner for XCMG XDE360",
        formatter_class=__import__("argparse").RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--start", required=True, metavar="X,Y,YAW",
                        help="Start pose: 'x_m,y_m,yaw_deg'")
    parser.add_argument("--goal", required=True, metavar="X,Y,YAW",
                        help="Goal pose: 'x_m,y_m,yaw_deg'")
    parser.add_argument("--via", action="append", metavar="X,Y,YAW",
                        help="Forced stop waypoint (repeatable). "
                             "Example: --via '80,30,45' --via '100,10,0'")
    parser.add_argument("--output", default="closed_loop/scenarios/planned_path.json",
                        help="Output JSON file "
                             "(default: closed_loop/scenarios/planned_path.json)")
    parser.add_argument("--plot", action="store_true",
                        help="Show matplotlib visualisation")
    parser.add_argument("--clothoid", action="store_true",
                        help="Replace circular arcs with full clothoid arcs "
                             "(G2-continuous steering, 2× longer arc segments)")
    parser.add_argument("--no-reverse", action="store_true",
                        help="Disable reverse segments (forward-only, Dubins-equivalent)")
    parser.add_argument("--casadi", action="store_true",
                        help="Refine path with CasADi/IPOPT trajectory optimiser "
                             "(requires: pip install casadi)")
    parser.add_argument("--casadi-dt", type=float, default=0.2,
                        help="CasADi integration timestep in seconds (default 0.2)")
    parser.add_argument("--rect-obstacle", action="append", metavar="X0,Y0,X1,Y1",
                        help="Rectangular obstacle in world coords (repeatable). "
                             "Example: --rect-obstacle '20,5,40,25'  "
                             "Forces A* routing when any obstacle is given.")
    parser.add_argument("--obstacles", default=None,
                        help="PNG obstacle map (white=free, black=obstacle)")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Obstacle map scale: metres per pixel (default 1.0)")
    parser.add_argument("--step", type=float, default=2.0,
                        help="Waypoint spacing in metres (default 2.0)")
    parser.add_argument("--grid-res", type=float, default=1.0,
                        help="A* grid resolution in metres (default 1.0)")

    _v = VehicleConfig()
    parser.add_argument("--wheelbase",     type=float, default=_v.wheelbase_m,
                        help=f"Vehicle wheelbase in m (default {_v.wheelbase_m})")
    parser.add_argument("--max-steer",     type=float, default=_v.max_steer_deg,
                        help=f"Max steering angle in deg (default {_v.max_steer_deg})")
    parser.add_argument("--fwd-speed",     type=float, default=_v.cruise_fwd_mps,
                        help=f"Forward cruise speed m/s (default {_v.cruise_fwd_mps})")
    parser.add_argument("--rev-speed",     type=float, default=_v.cruise_rev_mps,
                        help=f"Reverse cruise speed m/s (default {_v.cruise_rev_mps})")
    parser.add_argument("--arrive-radius", type=float, default=_v.arrive_radius_m,
                        help=f"Waypoint arrival radius in m (default {_v.arrive_radius_m})")

    args = parser.parse_args()

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
    sx, sy, syaw_deg = Geometry.parse_pose(args.start)
    gx, gy, gyaw_deg = Geometry.parse_pose(args.goal)
    syaw_rad = math.radians(syaw_deg)
    gyaw_rad = math.radians(gyaw_deg)
    print(f"  Start: ({sx}, {sy}, {syaw_deg}°)")
    print(f"  Goal:  ({gx}, {gy}, {gyaw_deg}°)")

    via_world: List[Tuple[float, float, float]] = []
    for v_str in (args.via or []):
        vx, vy, vyaw_deg = Geometry.parse_pose(v_str)
        via_world.append((vx, vy, math.radians(vyaw_deg)))
        print(f"  Via:   ({vx}, {vy}, {vyaw_deg}°)")

    # Rectangular obstacles (world-frame boxes)
    rect_obstacles: List[Tuple[float, float, float, float]] = []
    for r_str in (args.rect_obstacle or []):
        parts = [float(v.strip()) for v in r_str.split(",")]
        if len(parts) != 4:
            print(f"  WARNING: --rect-obstacle '{r_str}' ignored (need X0,Y0,X1,Y1)")
            continue
        rect_obstacles.append((parts[0], parts[1], parts[2], parts[3]))
        print(f"  Obstacle rect: ({parts[0]},{parts[1]}) → ({parts[2]},{parts[3]})")

    # PNG obstacle map (alternative to rect-obstacles)
    obstacle_map: Optional[np.ndarray] = None
    if args.obstacles and not rect_obstacles:
        try:
            from PIL import Image
            img          = np.array(Image.open(args.obstacles).convert("L"))
            obstacle_map = img < 128
            print(f"  Obstacle map: {obstacle_map.shape} @ {args.scale} m/px")
        except ImportError:
            print("  WARNING: Pillow not installed — ignoring obstacle map.")

    planner = PathPlanner(vehicle)
    try:
        scenario = planner.plan(
            start          = (sx, sy, syaw_rad),
            goal           = (gx, gy, gyaw_rad),
            via            = via_world or None,
            clothoid       = args.clothoid,
            no_reverse     = args.no_reverse,
            casadi         = args.casadi,
            casadi_dt      = args.casadi_dt,
            rect_obstacles = rect_obstacles or None,
            obstacle_map   = obstacle_map,
            grid_res       = args.grid_res,
            step           = args.step,
        )
    except RuntimeError as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

    json_str = json.dumps(scenario, indent=2)
    if args.output == "-":
        print(json_str)
    else:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json_str)
        print(f"  Saved: {args.output}")

    if args.plot:
        planner.plot_last(
            start=(sx, sy, syaw_rad),
            goal=(gx, gy, gyaw_rad),
        )

    print("Done.")


if __name__ == "__main__":
    main()
