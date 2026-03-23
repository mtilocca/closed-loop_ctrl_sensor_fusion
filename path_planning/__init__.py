"""
path_planning — A* + Reeds-Shepp + Clothoid offline path planner package.

Primary classes:
    VehicleConfig        — kinematic parameters (wheelbase, steering, speeds)
    ReedsSheppPlanner    — plan(), path_to_waypoints(), build_scenario_json(), plot()
    AStarPlanner         — plan() with optional obstacle map
    ClothoidSampler      — ClothoidSampler.sample() static method
    PathPlanner          — high-level orchestrator (chains legs, writes JSON)

Data types:
    Segment              — one RS arc/straight segment
    RSPath               — list of Segments + total_length; .sample(r_min, ...)
"""

from .helpers      import VehicleConfig, parse_pose                # noqa: F401
from .clothoid     import ClothoidSampler                          # noqa: F401
from .reeds_shepp  import RSPath, Segment, ReedsSheppPlanner       # noqa: F401
from .astar        import AStarPlanner                             # noqa: F401
from .main         import PathPlanner                              # noqa: F401
