#!/usr/bin/env python3
"""
path_planner.py — CLI entry point for the path_planning package.

Delegates to path_planning.main.  All logic lives in the package:
    path_planning/
        helpers.py      — Vehicle constants, geometry utilities
        clothoid.py     — Full clothoid arc (double Euler spiral) sampling
        reeds_shepp.py  — RS primitives, waypoint extraction, JSON output, plot
        astar.py        — A* on SE(2) lattice with RS motion primitives
        main.py         — CLI argument parsing and orchestration

Usage:
    python3 path_planner.py --start "0,0,0" --goal "60,40,90"
    python3 path_planner.py --start "0,0,0" --goal "60,40,90" --clothoid
    python3 path_planner.py --start "0,0,0" --via "80,30,45" --goal "120,0,0" --clothoid
    python3 path_planner.py --help
"""

from path_planning.main import main

if __name__ == "__main__":
    main()
