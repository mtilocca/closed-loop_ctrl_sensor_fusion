"""
path_planning — A* + Reeds-Shepp + Clothoid + CasADi offline path planner.

One class per module:

    geometry.py          → Geometry          (static angle/coord helpers)
    helpers.py           → VehicleConfig     (kinematic parameters)
    rs_path.py           → Segment, RSPath   (RS path data types + sampler)
    clothoid.py          → ClothoidSampler   (double Euler spiral arc sampler)
    reeds_shepp.py       → ReedsSheppPlanner (plan, path_to_waypoints, build_scenario_json, plot)
    astar.py             → AStarPlanner      (SE(2) A* with obstacle map)
    casadi_optimizer.py  → CasADiOptimizer   (IPOPT trajectory refinement)
    path_planner.py      → PathPlanner       (high-level orchestrator, multi-leg chaining)
    main.py              → main()            (CLI entry point)
"""

from .geometry          import Geometry                                    # noqa: F401
from .helpers           import VehicleConfig                               # noqa: F401
from .rs_path           import F, B, Segment, RSPath                      # noqa: F401
from .clothoid          import ClothoidSampler                             # noqa: F401
from .reeds_shepp       import ReedsSheppPlanner                           # noqa: F401
from .astar             import AStarPlanner                                # noqa: F401
from .casadi_optimizer  import CasADiOptimizer                             # noqa: F401
from .path_planner      import PathPlanner                                 # noqa: F401
