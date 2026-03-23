"""
reeds_shepp.py — Reeds-Shepp path planner, waypoint extraction, JSON builder,
                 and path visualisation.

Implements the 6 canonical RS path families from Reeds & Shepp (1990):
  CSC: LSL, RSR, LSR, RSL
  CCC: RLR, LRL

All 6 families plus their time-flip and reflect symmetry variants are
enumerated; the shortest valid path is returned.

Usage::

    vehicle = VehicleConfig()
    rs      = ReedsSheppPlanner(vehicle)
    path    = rs.plan(0, 0, 0, 50, 30, math.pi/2)
    wps     = rs.path_to_waypoints(path, 0, 0, 0)
    scenario = rs.build_scenario_json(wps, (0,0,0), (50,30,90))
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .geometry import Geometry
from .helpers  import VehicleConfig
from .rs_path  import F, B, Segment, RSPath


class ReedsSheppPlanner:
    """
    Plans shortest Reeds-Shepp paths, extracts waypoints, builds scenario JSON,
    and visualises results.

    Args:
        vehicle: VehicleConfig instance holding kinematic parameters.

    Example::

        vehicle = VehicleConfig(wheelbase_m=5.5, max_steer_deg=15.0)
        rs      = ReedsSheppPlanner(vehicle)
        path    = rs.plan(0, 0, 0, 50, 30, math.pi/2)
        wps     = rs.path_to_waypoints(path.sample(vehicle.r_min), 0, 0, 0)
        scenario = rs.build_scenario_json(wps, (0,0,0), (50,30,90))
    """

    def __init__(self, vehicle: VehicleConfig) -> None:
        self.vehicle = vehicle

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------

    def plan(
        self,
        sx: float, sy: float, syaw: float,
        gx: float, gy: float, gyaw: float,
        no_reverse: bool = False,
    ) -> Optional[RSPath]:
        """
        Compute the shortest Reeds-Shepp path from start to goal.

        Args:
            sx, sy, syaw: Start pose (metres, metres, radians).
            gx, gy, gyaw: Goal pose (metres, metres, radians).
            no_reverse:   If True, only forward-only (Dubins-equivalent) paths.

        Returns:
            RSPath, or None if no valid path exists.
        """
        r = self.vehicle.r_min
        lx, ly, lphi = self._transform_goal(sx, sy, syaw, gx, gy, gyaw, r)
        candidates   = self._all_rs_paths(lx, ly, lphi, r)
        if no_reverse:
            candidates = [p for p in candidates
                          if all(s.dir == F for s in p.segments)]
        if not candidates:
            return None
        return min(candidates, key=lambda p: p.total_length)

    def path_to_waypoints(
        self,
        poses:       List[Tuple[float, float, float, int]],
        sx:          float,
        sy:          float,
        syaw_rad:    float,
        sample_step: float = 2.0,
    ) -> List[dict]:
        """
        Convert sampled RS poses to waypoint dicts for the Go runner.

        Inserts zero-speed cusp waypoints at direction changes and assigns
        gear_position (1=Forward, 2=Reverse) and target_speed_mps.
        """
        v  = self.vehicle
        waypoints: List[dict] = []
        prev_dir      = None
        cusp_inserted = False
        last_x, last_y = sx, sy

        for i, (lx, ly, lyaw, direction) in enumerate(poses):
            c, s     = math.cos(syaw_rad), math.sin(syaw_rad)
            wx       = sx + c * lx - s * ly
            wy       = sy + s * lx + c * ly
            wyaw_deg = math.degrees(Geometry.wrap(syaw_rad + lyaw))
            dist     = math.hypot(wx - last_x, wy - last_y)

            if prev_dir is not None and direction != prev_dir and not cusp_inserted:
                waypoints.append({
                    "x_m":              round(wx, 3),
                    "y_m":              round(wy, 3),
                    "yaw_deg":          round(wyaw_deg, 2),
                    "target_speed_mps": v.cusp_speed_mps,
                    "gear_position":    1 if prev_dir == F else 2,
                    "arrive_radius_m":  v.arrive_radius_m,
                    "comment": f"cusp: {'fwd→rev' if prev_dir == F else 'rev→fwd'} gear change",
                })
                cusp_inserted = True
            else:
                cusp_inserted = False

            if (dist >= sample_step
                    or i == len(poses) - 1
                    or (prev_dir is not None and direction != prev_dir)):
                gear  = 1 if direction == F else 2
                speed = v.cruise_fwd_mps if direction == F else v.cruise_rev_mps
                waypoints.append({
                    "x_m":              round(wx, 3),
                    "y_m":              round(wy, 3),
                    "yaw_deg":          round(wyaw_deg, 2),
                    "target_speed_mps": speed,
                    "gear_position":    gear,
                    "arrive_radius_m":  v.arrive_radius_m,
                })
                last_x, last_y = wx, wy

            prev_dir = direction

        if waypoints:
            waypoints[-1]["target_speed_mps"] = 0.0
            waypoints[-1]["comment"]          = "goal: final stop"

        return waypoints

    def build_scenario_json(
        self,
        waypoints:   List[dict],
        start:       Tuple[float, float, float],
        goal:        Tuple[float, float, float],
        description: str = "",
    ) -> dict:
        """Build the scenario JSON dict consumable by the Go runner."""
        v  = self.vehicle
        sx, sy, syaw_deg = start
        gx, gy, gyaw_deg = goal
        n_wps     = len(waypoints)
        total_len = sum(
            math.hypot(waypoints[i]["x_m"] - waypoints[i-1]["x_m"],
                       waypoints[i]["y_m"] - waypoints[i-1]["y_m"])
            for i in range(1, n_wps)
        )
        mean_speed = (v.cruise_fwd_mps + v.cruise_rev_mps) / 2
        duration   = max(120.0, (total_len / mean_speed) * 2.5)

        return {
            "meta": {
                "name":         "path_to_goal",
                "version":      1,
                "description":  description or (
                    f"A* Reeds-Shepp path "
                    f"({sx:.1f},{sy:.1f},{syaw_deg:.0f}°) → "
                    f"({gx:.1f},{gy:.1f},{gyaw_deg:.0f}°) | "
                    f"{n_wps} waypoints | {total_len:.1f} m"
                ),
                "control_mode": "waypoint_pid",
            },
            "timing": {
                "dt_s":          0.01,
                "duration_s":    round(duration, 1),
                "log_hz":        10.0,
                "real_time_mode": True,
            },
            "pid_config": {
                "target_velocity_mps": v.cruise_fwd_mps,
                "kp":                  15000.0,
                "ki":                  800.0,
                "kd":                  3000.0,
                "max_torque_nm":       250000.0,
                "min_torque_nm":      -145000.0,
                "integral_limit":      5000.0,
            },
            "waypoints": waypoints,
        }

    def plot(
        self,
        poses:       List[Tuple[float, float, float, int]],
        sx:          float,
        sy:          float,
        syaw_rad:    float,
        gx:          float,
        gy:          float,
        gyaw_rad:    float,
        waypoints:   List[dict],
        poses_world: Optional[List[Tuple[float, float, float, int]]] = None,
        obstacles:   Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> None:
        """
        Plot the planned path with waypoints using matplotlib.

        Args:
            obstacles: Optional list of (x_min, y_min, x_max, y_max) rectangles
                       to draw as filled gray boxes.
        """
        try:
            import matplotlib.pyplot as plt
            from matplotlib.patches import FancyArrowPatch, Rectangle
        except ImportError:
            print("matplotlib not available — skipping plot.")
            return

        fig, ax   = plt.subplots(figsize=(14, 10))
        effective = poses_world if poses_world is not None else poses

        # Collect world-frame path points
        fwd_x, fwd_y, rev_x, rev_y = [], [], [], []
        all_wx, all_wy = [], []
        for lx, ly, _, direction in effective:
            if poses_world is not None:
                wx, wy = lx, ly
            else:
                c, s = math.cos(syaw_rad), math.sin(syaw_rad)
                wx = sx + c * lx - s * ly
                wy = sy + s * lx + c * ly
            all_wx.append(wx); all_wy.append(wy)
            (fwd_x if direction == F else rev_x).append(wx)
            (fwd_y if direction == F else rev_y).append(wy)

        # Draw obstacles first (behind path)
        if obstacles:
            for x0, y0, x1, y1 in obstacles:
                ax.add_patch(Rectangle(
                    (min(x0, x1), min(y0, y1)),
                    abs(x1 - x0), abs(y1 - y0),
                    facecolor="dimgray", edgecolor="black",
                    linewidth=1.0, alpha=0.6, zorder=2, label="_nolegend_",
                ))
            # Add a single legend entry for obstacles
            ax.add_patch(Rectangle((0, 0), 0, 0, facecolor="dimgray",
                                   alpha=0.6, label="Obstacle"))

        ax.plot(fwd_x, fwd_y, "b-", linewidth=2.0, alpha=0.8, label="Forward",  zorder=3)
        ax.plot(rev_x, rev_y, "r-", linewidth=2.0, alpha=0.8, label="Reverse",  zorder=3)

        for i, wp in enumerate(waypoints):
            colour = "limegreen" if wp["gear_position"] == 1 else "tomato"
            marker = "^"         if wp["gear_position"] == 1 else "v"
            ax.scatter(wp["x_m"], wp["y_m"], c=colour, marker=marker,
                       s=60, zorder=5, edgecolors="none")
            if "comment" in wp:
                ax.annotate(f"#{i} {wp['comment']}", (wp["x_m"], wp["y_m"]),
                            textcoords="offset points", xytext=(5, 5), fontsize=7)

        ax.scatter(sx, sy, c="black", marker="s", s=140, zorder=7, label="Start")
        ax.scatter(gx, gy, c="gold",  marker="*", s=250, zorder=7, label="Goal")

        arrow = max(5.0, self.vehicle.r_min * 0.3)
        for (px, py, pyaw, colour) in [
            (sx, sy, syaw_rad, "black"),
            (gx, gy, gyaw_rad, "goldenrod"),
        ]:
            ax.annotate(
                "", xy=(px + arrow * math.cos(pyaw), py + arrow * math.sin(pyaw)),
                xytext=(px, py),
                arrowprops=dict(arrowstyle="->", color=colour, lw=2.0),
                zorder=8,
            )

        # Auto-size axes to fit everything with padding
        all_x = list(all_wx) + [sx, gx] + ([x for r in (obstacles or []) for x in (r[0], r[2])])
        all_y = list(all_wy) + [sy, gy] + ([y for r in (obstacles or []) for y in (r[1], r[3])])
        if all_x and all_y:
            pad = max(10.0, (max(all_x) - min(all_x)) * 0.08,
                             (max(all_y) - min(all_y)) * 0.08)
            ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
            ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

        title = "Reeds-Shepp Path Plan"
        if poses_world is not None:
            title += " (clothoid arcs)"
        if obstacles:
            title += f" — {len(obstacles)} obstacle(s)"
        ax.set_xlabel("X East (m)")
        ax.set_ylabel("Y North (m)")
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
        plt.tight_layout()

        import tempfile, os
        png = os.path.join(tempfile.gettempdir(), "path_plan.png")
        fig.savefig(png, dpi=150, bbox_inches="tight")
        print(f"  Plot saved: {png}")

        try:
            plt.show()
        except Exception:
            pass

    # -----------------------------------------------------------------------
    # Private static methods — RS math primitives
    # -----------------------------------------------------------------------

    @staticmethod
    def _transform_goal(
        sx: float, sy: float, syaw: float,
        gx: float, gy: float, gyaw: float,
        r:  float,
    ) -> Tuple[float, float, float]:
        """Express goal in start frame, normalised by r_min."""
        dx, dy = gx - sx, gy - sy
        c, s   = math.cos(syaw), math.sin(syaw)
        lx = ( c * dx + s * dy) / r
        ly = (-s * dx + c * dy) / r
        return lx, ly, Geometry.wrap(gyaw - syaw)

    @staticmethod
    def _seg(length: float, turn: float, direction: int, r: float) -> Optional[Segment]:
        if length < 0:
            return None
        return Segment(length * r, turn, direction)

    @staticmethod
    def _build_path(*segs: Segment) -> RSPath:
        return RSPath(list(segs), sum(s.length for s in segs))

    # CSC families

    @staticmethod
    def _lsl(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
        _RS = ReedsSheppPlanner
        u, t = Geometry.polar(x - math.sin(phi), y - 1 + math.cos(phi))
        if u < 0:
            return None
        v = Geometry.wrap(phi - t)
        s0, s1, s2 = _RS._seg(t, +1, F, r), _RS._seg(u, 0, F, r), _RS._seg(v, +1, F, r)
        if any(s is None for s in (s0, s1, s2)):
            return None
        return _RS._build_path(s0, s1, s2)

    @staticmethod
    def _rsr(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
        _RS = ReedsSheppPlanner
        u, t = Geometry.polar(x + math.sin(phi), y - 1 - math.cos(phi))
        v = Geometry.wrap(t - phi)
        s0, s1, s2 = _RS._seg(t, -1, F, r), _RS._seg(u, 0, F, r), _RS._seg(v, -1, F, r)
        if any(s is None for s in (s0, s1, s2)):
            return None
        return _RS._build_path(s0, s1, s2)

    @staticmethod
    def _lsr(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
        _RS = ReedsSheppPlanner
        u1sq = x**2 + y**2 - 2*y + 2*x*math.sin(phi) - 2*y*math.cos(phi) + 1
        if u1sq < 0:
            return None
        u1 = math.sqrt(u1sq)
        t  = Geometry.wrap(math.atan2(
            -math.cos(phi) - (y - 1) / u1,
            (x - math.sin(phi)) / u1,
        ))
        v  = Geometry.wrap(t - phi)
        s0, s1, s2 = _RS._seg(t, +1, F, r), _RS._seg(u1, 0, F, r), _RS._seg(v, -1, F, r)
        if any(s is None for s in (s0, s1, s2)):
            return None
        return _RS._build_path(s0, s1, s2)

    @staticmethod
    def _rsl(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
        _RS = ReedsSheppPlanner
        u1sq = x**2 + y**2 + 2*y - 2*x*math.sin(phi) + 2*y*math.cos(phi) + 1
        if u1sq < 0:
            return None
        u1 = math.sqrt(u1sq)
        t  = Geometry.wrap(math.atan2(
            math.cos(phi) + (y + 1) / u1,
            (x + math.sin(phi)) / u1,
        ))
        v  = Geometry.wrap(phi - t)
        s0, s1, s2 = _RS._seg(t, -1, F, r), _RS._seg(u1, 0, F, r), _RS._seg(v, +1, F, r)
        if any(s is None for s in (s0, s1, s2)):
            return None
        return _RS._build_path(s0, s1, s2)

    # CCC families

    @staticmethod
    def _rlr(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
        _RS = ReedsSheppPlanner
        u1, theta = Geometry.polar(x - math.sin(phi), y - 1 + math.cos(phi))
        if u1 > 4:
            return None
        A = math.acos(u1 / 4)
        t = Geometry.mod2pi(theta + A + math.pi / 2)
        u = Geometry.mod2pi(math.pi - 2 * A)
        v = Geometry.mod2pi(phi - t - u)
        s0, s1, s2 = _RS._seg(t, -1, F, r), _RS._seg(u, +1, F, r), _RS._seg(v, -1, F, r)
        if any(s is None for s in (s0, s1, s2)):
            return None
        return _RS._build_path(s0, s1, s2)

    @staticmethod
    def _lrl(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
        _RS = ReedsSheppPlanner
        u1, theta = Geometry.polar(x + math.sin(phi), y - 1 - math.cos(phi))
        if u1 > 4:
            return None
        A = math.acos(u1 / 4)
        t = Geometry.mod2pi(theta - A - math.pi / 2)
        u = Geometry.mod2pi(math.pi - 2 * A)
        v = Geometry.mod2pi(t + u - phi)
        s0, s1, s2 = _RS._seg(t, +1, F, r), _RS._seg(u, -1, F, r), _RS._seg(v, +1, F, r)
        if any(s is None for s in (s0, s1, s2)):
            return None
        return _RS._build_path(s0, s1, s2)

    @staticmethod
    def _all_rs_paths(x: float, y: float, phi: float, r: float) -> List[RSPath]:
        """Enumerate all RS candidates (6 families × 4 symmetry variants)."""
        _RS = ReedsSheppPlanner
        _families = [_RS._lsl, _RS._rsr, _RS._lsr, _RS._rsl,
                     _RS._rlr, _RS._lrl]
        candidates: List[RSPath] = []
        for fn in _families:
            for xi, yi, pi in [
                ( x,  y,  phi),
                (-x,  y, -phi),   # timeflip
                ( x, -y, -phi),   # reflect
                (-x, -y,  phi),   # timeflip + reflect
            ]:
                p = fn(xi, yi, pi, r)
                if p is not None:
                    candidates.append(p)
        return candidates
