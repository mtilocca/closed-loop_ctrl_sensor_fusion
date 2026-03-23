"""
reeds_shepp.py — Reeds-Shepp path primitives, waypoint extraction, JSON output,
                 and path visualisation.

Implements the 6 canonical RS path families from Reeds & Shepp (1990):
  CSC: LSL, RSR, LSR, RSL
  CCC: RLR, LRL

Reverse-gear variants are generated via time-flip and reflect symmetries.

Main entry point::

    planner = ReedsSheppPlanner(vehicle)
    path    = planner.plan(sx, sy, syaw, gx, gy, gyaw)
    wps     = planner.path_to_waypoints(path, sx, sy, syaw_rad)
    scenario = planner.build_scenario_json(wps, start, goal)
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from .helpers import VehicleConfig, wrap, mod2pi, polar

# Direction constants
F = +1   # forward
B = -1   # backward (reverse gear)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class Segment:
    length: float     # arc length in metres (positive)
    turn:   float     # +1 = left, -1 = right, 0 = straight
    dir:    int       # +1 = forward, -1 = backward


@dataclass
class RSPath:
    segments:     List[Segment]
    total_length: float

    def sample(
        self,
        r_min:    float,
        step:     float = 0.5,
        clothoid: bool  = False,
    ) -> List[Tuple[float, float, float, int]]:
        """
        Sample path as a list of (x, y, yaw_rad, direction) poses.

        Args:
            r_min:    Minimum turning radius (metres).
            step:     Sampling interval in metres.
            clothoid: If True, replace circular arcs with full clothoid arcs
                      (double Euler spirals) for G2-continuous curvature.
        """
        from .clothoid import ClothoidSampler

        poses: List[Tuple[float, float, float, int]] = []
        x, y, yaw = 0.0, 0.0, 0.0

        for seg in self.segments:
            if seg.turn == 0:                          # straight — unchanged
                dist = seg.length
                n = max(1, int(dist / step))
                ds = dist / n
                for _ in range(n):
                    poses.append((x, y, yaw, seg.dir))
                    x += seg.dir * ds * math.cos(yaw)
                    y += seg.dir * ds * math.sin(yaw)

            elif clothoid:                              # full clothoid arc
                arc_poses = ClothoidSampler.sample(
                    x, y, yaw, seg.length, seg.turn, seg.dir, r_min, step
                )
                poses.extend(arc_poses)
                x, y, yaw = arc_poses[-1][0], arc_poses[-1][1], arc_poses[-1][2]

            else:                                       # RS circular arc
                dist = seg.length
                n = max(1, int(dist / step))
                ds = dist / n
                for _ in range(n):
                    poses.append((x, y, yaw, seg.dir))
                    dtheta = seg.dir * seg.turn * ds / r_min
                    cx  = x - r_min * seg.turn * math.sin(yaw)
                    cy  = y + r_min * seg.turn * math.cos(yaw)
                    yaw2 = yaw + dtheta
                    x   = cx + r_min * seg.turn * math.sin(yaw2)
                    y   = cy - r_min * seg.turn * math.cos(yaw2)
                    yaw = wrap(yaw2)

        poses.append((x, y, yaw, poses[-1][3] if poses else F))
        return poses


# ---------------------------------------------------------------------------
# RS math primitives (module-level, pure — r passed explicitly)
# ---------------------------------------------------------------------------

def _transform_goal(
    sx: float, sy: float, syaw: float,
    gx: float, gy: float, gyaw: float,
    r: float,
) -> Tuple[float, float, float]:
    """Express goal in start frame, normalised by r_min."""
    dx, dy = gx - sx, gy - sy
    c, s   = math.cos(syaw), math.sin(syaw)
    lx = ( c * dx + s * dy) / r
    ly = (-s * dx + c * dy) / r
    return lx, ly, wrap(gyaw - syaw)


def _seg(length: float, turn: float, direction: int, r: float) -> Optional[Segment]:
    if length < 0:
        return None
    return Segment(length * r, turn, direction)


def _build_path(*segs: Segment) -> RSPath:
    total = sum(s.length for s in segs)
    return RSPath(list(segs), total)


# CSC families ---------------------------------------------------------------

def _lsl(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
    u, t = polar(x - math.sin(phi), y - 1 + math.cos(phi))
    if u < 0:
        return None
    v = wrap(phi - t)
    s0, s1, s2 = _seg(t, +1, F, r), _seg(u, 0, F, r), _seg(v, +1, F, r)
    if any(s is None for s in [s0, s1, s2]):
        return None
    return _build_path(s0, s1, s2)


def _rsr(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
    u, t = polar(x + math.sin(phi), y - 1 - math.cos(phi))
    v = wrap(t - phi)
    s0, s1, s2 = _seg(t, -1, F, r), _seg(u, 0, F, r), _seg(v, -1, F, r)
    if any(s is None for s in [s0, s1, s2]):
        return None
    return _build_path(s0, s1, s2)


def _lsr(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
    u1sq = (x**2 + y**2 - 2*y + 2*x*math.sin(phi) - 2*y*math.cos(phi) + 1)
    if u1sq < 0:
        return None
    u1 = math.sqrt(u1sq)
    t1 = math.atan2(-math.cos(phi) - (y - 1) / u1,
                    (x - math.sin(phi)) / u1)
    t  = wrap(t1)
    v  = wrap(t - phi)
    s0, s1, s2 = _seg(t, +1, F, r), _seg(u1, 0, F, r), _seg(v, -1, F, r)
    if any(s is None for s in [s0, s1, s2]):
        return None
    return _build_path(s0, s1, s2)


def _rsl(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
    u1sq = (x**2 + y**2 + 2*y + 2*x*(-math.sin(phi)) + 2*y*math.cos(phi) + 1)
    if u1sq < 0:
        return None
    u1 = math.sqrt(u1sq)
    t1 = math.atan2(math.cos(phi) + (y + 1) / u1,
                    (x + math.sin(phi)) / u1)
    t  = wrap(t1)
    v  = wrap(phi - t)
    s0, s1, s2 = _seg(t, -1, F, r), _seg(u1, 0, F, r), _seg(v, +1, F, r)
    if any(s is None for s in [s0, s1, s2]):
        return None
    return _build_path(s0, s1, s2)


# CCC families ---------------------------------------------------------------

def _rlr(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
    xi, eta = x - math.sin(phi), y - 1 + math.cos(phi)
    u1, theta = polar(xi, eta)
    if u1 > 4:
        return None
    A = math.acos(u1 / 4)
    t = mod2pi(theta + A + math.pi / 2)
    u = mod2pi(math.pi - 2 * A)
    v = mod2pi(phi - t - u)
    s0, s1, s2 = _seg(t, -1, F, r), _seg(u, +1, F, r), _seg(v, -1, F, r)
    if any(s is None for s in [s0, s1, s2]):
        return None
    return _build_path(s0, s1, s2)


def _lrl(x: float, y: float, phi: float, r: float) -> Optional[RSPath]:
    xi, eta = x + math.sin(phi), y - 1 - math.cos(phi)
    u1, theta = polar(xi, eta)
    if u1 > 4:
        return None
    A = math.acos(u1 / 4)
    t = mod2pi(theta - A - math.pi / 2)
    u = mod2pi(math.pi - 2 * A)
    v = mod2pi(t + u - phi)
    s0, s1, s2 = _seg(t, +1, F, r), _seg(u, -1, F, r), _seg(v, +1, F, r)
    if any(s is None for s in [s0, s1, s2]):
        return None
    return _build_path(s0, s1, s2)


# Symmetry variants ----------------------------------------------------------

def _timeflip(p: RSPath) -> RSPath:
    segs = [Segment(s.length, s.turn, -s.dir) for s in reversed(p.segments)]
    return RSPath(segs, p.total_length)


def _reflect(p: RSPath) -> RSPath:
    segs = [Segment(s.length, -s.turn, s.dir) for s in p.segments]
    return RSPath(segs, p.total_length)


_CSC_FUNCS = [_lsl, _rsr, _lsr, _rsl]
_CCC_FUNCS = [_rlr, _lrl]


def _all_rs_paths(x: float, y: float, phi: float, r: float) -> List[RSPath]:
    """Generate all RS candidate paths for normalised goal (x, y, phi)."""
    candidates: List[RSPath] = []
    for fn in _CSC_FUNCS + _CCC_FUNCS:
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


# ---------------------------------------------------------------------------
# ReedsSheppPlanner
# ---------------------------------------------------------------------------

class ReedsSheppPlanner:
    """
    Plans shortest Reeds-Shepp paths, extracts waypoints, and builds
    scenario JSON for the Go runner.

    Args:
        vehicle: VehicleConfig instance holding kinematic parameters.

    Example::

        vehicle = VehicleConfig(wheelbase_m=5.5, max_steer_deg=15.0)
        rs      = ReedsSheppPlanner(vehicle)
        path    = rs.plan(0, 0, 0, 50, 30, math.pi/2)
        wps     = rs.path_to_waypoints(path, 0, 0, 0)
        scenario = rs.build_scenario_json(wps, (0,0,0), (50,30,90))
    """

    def __init__(self, vehicle: VehicleConfig) -> None:
        self.vehicle = vehicle

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

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
            no_reverse:   If True, only forward-only paths (Dubins-equivalent).

        Returns:
            RSPath or None if no valid path exists.
        """
        r = self.vehicle.r_min
        lx, ly, lphi = _transform_goal(sx, sy, syaw, gx, gy, gyaw, r)
        candidates = _all_rs_paths(lx, ly, lphi, r)
        if no_reverse:
            candidates = [p for p in candidates
                          if all(s.dir == F for s in p.segments)]
        if not candidates:
            return None
        return min(candidates, key=lambda p: p.total_length)

    # ------------------------------------------------------------------
    # Waypoint extraction
    # ------------------------------------------------------------------

    def path_to_waypoints(
        self,
        poses:       List[Tuple[float, float, float, int]],
        sx:          float,
        sy:          float,
        syaw_rad:    float,
        sample_step: float = 2.0,
    ) -> List[dict]:
        """
        Convert sampled RS path poses to waypoint dicts for the Go runner.

        Inserts zero-speed cusp waypoints at direction changes.
        Assigns gear_position (1=Forward, 2=Reverse) and target_speed_mps.
        """
        v = self.vehicle
        waypoints: List[dict] = []
        prev_dir = None
        cusp_inserted = False
        last_x, last_y = sx, sy

        for i, (lx, ly, lyaw, direction) in enumerate(poses):
            c, s = math.cos(syaw_rad), math.sin(syaw_rad)
            wx = sx + c * lx - s * ly
            wy = sy + s * lx + c * ly
            wyaw_deg = math.degrees(wrap(syaw_rad + lyaw))
            dist = math.hypot(wx - last_x, wy - last_y)

            # Direction change → insert cusp waypoint
            if prev_dir is not None and direction != prev_dir and not cusp_inserted:
                waypoints.append({
                    "x_m":             round(wx, 3),
                    "y_m":             round(wy, 3),
                    "yaw_deg":         round(wyaw_deg, 2),
                    "target_speed_mps": v.cusp_speed_mps,
                    "gear_position":   1 if prev_dir == F else 2,
                    "arrive_radius_m": v.arrive_radius_m,
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
                    "x_m":             round(wx, 3),
                    "y_m":             round(wy, 3),
                    "yaw_deg":         round(wyaw_deg, 2),
                    "target_speed_mps": speed,
                    "gear_position":   gear,
                    "arrive_radius_m": v.arrive_radius_m,
                })
                last_x, last_y = wx, wy

            prev_dir = direction

        if waypoints:
            waypoints[-1]["target_speed_mps"] = 0.0
            waypoints[-1]["comment"] = "goal: final stop"

        return waypoints

    # ------------------------------------------------------------------
    # Scenario JSON builder
    # ------------------------------------------------------------------

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
        n_wps = len(waypoints)
        total_len = sum(
            math.hypot(waypoints[i]["x_m"] - waypoints[i-1]["x_m"],
                       waypoints[i]["y_m"] - waypoints[i-1]["y_m"])
            for i in range(1, n_wps)
        )
        mean_speed = (v.cruise_fwd_mps + v.cruise_rev_mps) / 2
        duration   = max(120.0, (total_len / mean_speed) * 2.5)

        return {
            "meta": {
                "name": "path_to_goal",
                "version": 1,
                "description": description or (
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
                "kp":               15000.0,
                "ki":               800.0,
                "kd":               3000.0,
                "max_torque_nm":    250000.0,
                "min_torque_nm":   -145000.0,
                "integral_limit":  5000.0,
            },
            "waypoints": waypoints,
        }

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

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
    ) -> None:
        """Plot the planned path with waypoints."""
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available — skipping plot.")
            return

        fig, ax = plt.subplots(figsize=(12, 10))
        effective = poses_world if poses_world is not None else poses

        fwd_x, fwd_y, rev_x, rev_y = [], [], [], []
        for item in effective:
            lx, ly, _, direction = item
            if poses_world is not None:
                wx, wy = lx, ly
            else:
                c, s = math.cos(syaw_rad), math.sin(syaw_rad)
                wx = sx + c * lx - s * ly
                wy = sy + s * lx + c * ly
            (fwd_x if direction == F else rev_x).append(wx)
            (fwd_y if direction == F else rev_y).append(wy)

        ax.plot(fwd_x, fwd_y, "b-", linewidth=2.0, alpha=0.7, label="Forward")
        ax.plot(rev_x, rev_y, "r-", linewidth=2.0, alpha=0.7, label="Reverse")

        for i, wp in enumerate(waypoints):
            colour = "green" if wp["gear_position"] == 1 else "red"
            marker = "^"     if wp["gear_position"] == 1 else "v"
            ax.scatter(wp["x_m"], wp["y_m"], c=colour, marker=marker, s=80, zorder=5)
            if "comment" in wp:
                ax.annotate(
                    f"#{i} {wp['comment']}",
                    (wp["x_m"], wp["y_m"]),
                    textcoords="offset points", xytext=(5, 5), fontsize=7,
                )

        ax.scatter(sx, sy, c="black", marker="s", s=120, zorder=6, label="Start")
        ax.scatter(gx, gy, c="gold",  marker="*", s=200, zorder=6, label="Goal")

        arrow_len = max(5.0, self.vehicle.r_min * 0.3)
        ax.annotate("", xy=(sx + arrow_len * math.cos(syaw_rad),
                             sy + arrow_len * math.sin(syaw_rad)),
                    xytext=(sx, sy),
                    arrowprops=dict(arrowstyle="->", color="black", lw=1.5))
        ax.annotate("", xy=(gx + arrow_len * math.cos(gyaw_rad),
                             gy + arrow_len * math.sin(gyaw_rad)),
                    xytext=(gx, gy),
                    arrowprops=dict(arrowstyle="->", color="goldenrod", lw=1.5))

        ax.set_xlabel("X East (m)")
        ax.set_ylabel("Y North (m)")
        ax.set_title("Reeds-Shepp Path Plan"
                     + (" (clothoid arcs)" if poses_world is not None else ""))
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
        plt.tight_layout()
        plt.show()


# ---------------------------------------------------------------------------
# Back-compat module-level functions
# ---------------------------------------------------------------------------

def reeds_shepp(
    sx: float, sy: float, syaw: float,
    gx: float, gy: float, gyaw: float,
    no_reverse: bool = False,
    _vehicle: Optional[VehicleConfig] = None,
) -> Optional[RSPath]:
    """Module-level wrapper for ReedsSheppPlanner.plan (back-compat)."""
    from .helpers import VehicleConfig as _VC
    v = _vehicle or _VC()
    return ReedsSheppPlanner(v).plan(sx, sy, syaw, gx, gy, gyaw, no_reverse)


def path_to_waypoints(
    poses:       List[Tuple[float, float, float, int]],
    sx:          float,
    sy:          float,
    syaw_rad:    float,
    sample_step: float = 2.0,
    _vehicle:    Optional[VehicleConfig] = None,
) -> List[dict]:
    """Module-level wrapper for ReedsSheppPlanner.path_to_waypoints (back-compat)."""
    from .helpers import VehicleConfig as _VC
    v = _vehicle or _VC()
    return ReedsSheppPlanner(v).path_to_waypoints(poses, sx, sy, syaw_rad, sample_step)


def build_scenario_json(
    waypoints:   List[dict],
    start:       Tuple[float, float, float],
    goal:        Tuple[float, float, float],
    description: str = "",
    _vehicle:    Optional[VehicleConfig] = None,
) -> dict:
    """Module-level wrapper for ReedsSheppPlanner.build_scenario_json (back-compat)."""
    from .helpers import VehicleConfig as _VC
    v = _vehicle or _VC()
    return ReedsSheppPlanner(v).build_scenario_json(waypoints, start, goal, description)


def plot_path(
    poses:       List[Tuple[float, float, float, int]],
    sx:          float,
    sy:          float,
    syaw_rad:    float,
    gx:          float,
    gy:          float,
    gyaw_rad:    float,
    waypoints:   List[dict],
    poses_world: Optional[List[Tuple[float, float, float, int]]] = None,
    _vehicle:    Optional[VehicleConfig] = None,
) -> None:
    """Module-level wrapper for ReedsSheppPlanner.plot (back-compat)."""
    from .helpers import VehicleConfig as _VC
    v = _vehicle or _VC()
    ReedsSheppPlanner(v).plot(
        poses, sx, sy, syaw_rad, gx, gy, gyaw_rad, waypoints, poses_world
    )
