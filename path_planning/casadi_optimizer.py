"""
casadi_optimizer.py — CasADi/IPOPT trajectory optimiser for smooth,
                      kinematically feasible waypoint refinement.

Takes a reference waypoint list (from ReedsSheppPlanner or AStarPlanner) as
initial guess and solves a nonlinear optimisation problem over the bicycle
model to produce a smooth, minimum-jerk trajectory.

Usage::

    from path_planning.helpers          import VehicleConfig
    from path_planning.casadi_optimizer import CasADiOptimizer

    vehicle   = VehicleConfig()
    optimizer = CasADiOptimizer(vehicle)
    refined   = optimizer.optimize(waypoints, dt=0.2)
    # refined is a list[dict] with the same schema as the RS planner output

Requires: pip install casadi
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .helpers import VehicleConfig


class CasADiOptimizer:
    """
    Trajectory optimiser using CasADi's Opti stack with IPOPT.

    The bicycle kinematic model is used as the equality constraint::

        x_{k+1}   = x_k   + v_k * cos(yaw_k) * dt
        y_{k+1}   = y_k   + v_k * sin(yaw_k) * dt
        yaw_{k+1} = yaw_k + v_k * tan(delta_k) / L * dt
        v_{k+1}   = v_k   + a_k * dt

    Objective (all terms weighted)::

        min  Σ_k [ w_a    * a_k²
                 + w_d    * delta_k²
                 + w_dr   * (delta_{k+1} − delta_k)²   ← steering rate
                 + w_xy   * ‖(x_k,y_k) − ref_k‖²       ← path tracking
                 + w_yaw  * (yaw_k − ref_yaw_k)²  ]

    Constraints::

        |a_k|     ≤ a_max
        |delta_k| ≤ delta_max   (from vehicle.max_steer_deg)
        0 ≤ v_k   ≤ v_max       (speed envelope)
        boundary conditions fixed to first/last reference waypoints

    Args:
        vehicle:       VehicleConfig holding kinematic limits.
        w_tracking:    Weight on lateral deviation from reference path (default 1.0).
        w_steer:       Weight on absolute steering angle (default 0.1).
        w_steer_rate:  Weight on steering rate — penalises jumps (default 2.0).
        w_accel:       Weight on acceleration magnitude (default 0.05).
        a_max:         Peak longitudinal acceleration in m/s² (default 2.0).
        v_max:         Maximum forward speed in m/s (default vehicle.cruise_fwd_mps).
        solver_opts:   Dict of IPOPT options to forward to CasADi (optional).
    """

    def __init__(
        self,
        vehicle:      VehicleConfig,
        w_tracking:   float = 1.0,
        w_steer:      float = 0.1,
        w_steer_rate: float = 2.0,
        w_accel:      float = 0.05,
        a_max:        float = 2.0,
        v_max:        Optional[float] = None,
        solver_opts:  Optional[dict] = None,
    ) -> None:
        self.vehicle      = vehicle
        self.w_tracking   = w_tracking
        self.w_steer      = w_steer
        self.w_steer_rate = w_steer_rate
        self.w_accel      = w_accel
        self.a_max        = a_max
        self.v_max        = v_max if v_max is not None else vehicle.cruise_fwd_mps
        self.solver_opts  = solver_opts or {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def optimize(
        self,
        waypoints: List[dict],
        dt:        float = 0.2,
        N:         Optional[int] = None,
    ) -> List[dict]:
        """
        Refine a waypoint list with the IPOPT trajectory optimiser.

        Args:
            waypoints: Reference waypoints produced by ReedsSheppPlanner or
                       AStarPlanner.  Must contain x_m, y_m, yaw_deg keys.
            dt:        Integration timestep in seconds (default 0.2).
            N:         Number of optimisation knot points.  Defaults to
                       len(waypoints) capped at 200 for tractability.

        Returns:
            Refined waypoint list with the same dict schema as the input,
            ready to drop into a Go runner scenario JSON.

        Raises:
            ImportError: If CasADi is not installed.
            RuntimeError: If IPOPT fails to find a feasible solution.
        """
        try:
            import casadi as ca
        except ImportError:
            raise ImportError(
                "CasADi is not installed.  Install it with:\n"
                "    pip install casadi"
            )

        if len(waypoints) < 2:
            return waypoints

        # Clamp N for tractability
        N_ref = len(waypoints)
        N     = N or min(N_ref, 200)

        # Build reference arrays (linearly interpolated from waypoints)
        ref = self._interpolate_reference(waypoints, N)
        ref_x   = [p[0] for p in ref]
        ref_y   = [p[1] for p in ref]
        ref_yaw = [p[2] for p in ref]
        ref_v   = [p[3] for p in ref]

        # ---- Opti stack setup ------------------------------------------
        opti = ca.Opti()

        # State:   x, y, yaw, v  at each knot  (N+1 points)
        X  = opti.variable(4, N + 1)   # rows: x, y, yaw, v
        # Control: a, delta          at each step (N points)
        U  = opti.variable(2, N)       # rows: a, delta

        x_var   = X[0, :]
        y_var   = X[1, :]
        yaw_var = X[2, :]
        v_var   = X[3, :]
        a_var   = U[0, :]
        d_var   = U[1, :]

        L           = self.vehicle.wheelbase_m
        delta_max   = math.radians(self.vehicle.max_steer_deg)

        # ---- Objective --------------------------------------------------
        obj = 0
        for k in range(N):
            dx   = x_var[k]   - ref_x[k]
            dy   = y_var[k]   - ref_y[k]
            dyaw = yaw_var[k] - ref_yaw[k]
            obj += (self.w_tracking  * (dx**2 + dy**2)
                  + self.w_steer     * d_var[k]**2
                  + self.w_accel     * a_var[k]**2)
            if k > 0:
                obj += self.w_steer_rate * (d_var[k] - d_var[k-1])**2

        opti.minimize(obj)

        # ---- Dynamics constraints ---------------------------------------
        for k in range(N):
            vk  = v_var[k]
            yk  = yaw_var[k]
            opti.subject_to(
                x_var[k+1]   == x_var[k]   + vk * ca.cos(yk) * dt
            )
            opti.subject_to(
                y_var[k+1]   == y_var[k]   + vk * ca.sin(yk) * dt
            )
            opti.subject_to(
                yaw_var[k+1] == yaw_var[k] + vk * ca.tan(d_var[k]) / L * dt
            )
            opti.subject_to(
                v_var[k+1]   == v_var[k]   + a_var[k] * dt
            )

        # ---- Box constraints --------------------------------------------
        opti.subject_to(opti.bounded(-self.a_max,  a_var, self.a_max))
        opti.subject_to(opti.bounded(-delta_max,   d_var, delta_max))
        opti.subject_to(opti.bounded(0.0,          v_var, self.v_max))

        # ---- Boundary conditions ----------------------------------------
        opti.subject_to(x_var[0]   == ref_x[0])
        opti.subject_to(y_var[0]   == ref_y[0])
        opti.subject_to(yaw_var[0] == ref_yaw[0])
        opti.subject_to(v_var[0]   == ref_v[0])
        opti.subject_to(x_var[N]   == ref_x[-1])
        opti.subject_to(y_var[N]   == ref_y[-1])
        opti.subject_to(v_var[N]   == 0.0)         # come to a stop at goal

        # ---- Initial guess from reference --------------------------------
        opti.set_initial(x_var,   ref_x[:N+1])
        opti.set_initial(y_var,   ref_y[:N+1])
        opti.set_initial(yaw_var, ref_yaw[:N+1])
        opti.set_initial(v_var,   ref_v[:N+1])
        opti.set_initial(a_var,   [0.0] * N)
        opti.set_initial(d_var,   [0.0] * N)

        # ---- Solver options ----------------------------------------------
        opts: dict = {
            "ipopt.print_level":        0,
            "ipopt.sb":                 "yes",
            "print_time":               False,
            "ipopt.max_iter":           500,
            "ipopt.tol":                1e-6,
        }
        opts.update(self.solver_opts)
        opti.solver("ipopt", opts)

        # ---- Solve -------------------------------------------------------
        try:
            sol = opti.solve()
        except RuntimeError as exc:
            raise RuntimeError(
                f"CasADi/IPOPT failed to converge: {exc}\n"
                "Try increasing dt, reducing N, or relaxing weights."
            ) from exc

        # ---- Extract solution and build waypoint list --------------------
        xs   = sol.value(x_var).tolist()
        ys   = sol.value(y_var).tolist()
        yaws = sol.value(yaw_var).tolist()
        vs   = sol.value(v_var).tolist()

        v = self.vehicle
        result: List[dict] = []
        for k in range(N + 1):
            wp: dict = {
                "x_m":              round(xs[k], 3),
                "y_m":              round(ys[k], 3),
                "yaw_deg":          round(math.degrees(yaws[k]), 2),
                "target_speed_mps": round(max(0.0, vs[k]), 3),
                "gear_position":    1,
                "arrive_radius_m":  v.arrive_radius_m,
            }
            result.append(wp)

        result[-1]["target_speed_mps"] = 0.0
        result[-1]["comment"]          = "goal: final stop (casadi)"
        return result

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _interpolate_reference(
        self,
        waypoints: List[dict],
        N:         int,
    ) -> List[Tuple[float, float, float, float]]:
        """
        Linearly interpolate the reference waypoints to exactly N+1 knots.

        Returns list of (x, y, yaw_rad, v_mps) tuples.
        """
        pts = [(wp["x_m"],
                wp["y_m"],
                math.radians(wp.get("yaw_deg", 0.0)),
                wp.get("target_speed_mps", self.vehicle.cruise_fwd_mps))
               for wp in waypoints]

        # Compute cumulative arc-length along reference
        s = [0.0]
        for i in range(1, len(pts)):
            ds = math.hypot(pts[i][0] - pts[i-1][0],
                            pts[i][1] - pts[i-1][1])
            s.append(s[-1] + ds)
        total = s[-1] if s[-1] > 0 else 1.0

        result: List[Tuple[float, float, float, float]] = []
        for k in range(N + 1):
            sk  = total * k / N
            # Find bracketing segment
            idx = 0
            for j in range(len(s) - 1):
                if s[j] <= sk <= s[j+1]:
                    idx = j
                    break
            else:
                idx = len(pts) - 2

            ds_seg = s[idx+1] - s[idx]
            t      = (sk - s[idx]) / ds_seg if ds_seg > 0 else 0.0
            t      = max(0.0, min(1.0, t))

            x   = pts[idx][0] + t * (pts[idx+1][0] - pts[idx][0])
            y   = pts[idx][1] + t * (pts[idx+1][1] - pts[idx][1])
            yaw = pts[idx][2] + t * (pts[idx+1][2] - pts[idx][2])
            v   = pts[idx][3] + t * (pts[idx+1][3] - pts[idx][3])
            result.append((x, y, yaw, v))

        return result
