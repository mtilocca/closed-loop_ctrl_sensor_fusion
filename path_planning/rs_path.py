"""
rs_path.py — Reeds-Shepp path data types.

Segment  — one arc or straight piece of an RS path.
RSPath   — ordered list of Segments with sampling support.

These types are consumed by ReedsSheppPlanner and AStarPlanner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from .geometry import Geometry

# Direction constants
F = +1   # forward
B = -1   # backward (reverse gear)


@dataclass
class Segment:
    """One piece of a Reeds-Shepp path."""
    length: float   # arc length in metres (positive)
    turn:   float   # +1 = left, -1 = right, 0 = straight
    dir:    int     # +1 = forward, -1 = backward


@dataclass
class RSPath:
    """
    A Reeds-Shepp path: an ordered list of Segments plus total arc length.

    Use ReedsSheppPlanner.plan() to obtain an RSPath instance.
    """
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
                      Arc segments become 2× longer but steering is smooth.

        Returns:
            List of (x_m, y_m, yaw_rad, direction) tuples in the path-local frame.
        """
        from .clothoid import ClothoidSampler

        poses: List[Tuple[float, float, float, int]] = []
        x, y, yaw = 0.0, 0.0, 0.0

        for seg in self.segments:
            if seg.turn == 0:                          # straight — unchanged
                n  = max(1, int(seg.length / step))
                ds = seg.length / n
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
                n  = max(1, int(seg.length / step))
                ds = seg.length / n
                for _ in range(n):
                    poses.append((x, y, yaw, seg.dir))
                    dtheta = seg.dir * seg.turn * ds / r_min
                    cx     = x - r_min * seg.turn * math.sin(yaw)
                    cy     = y + r_min * seg.turn * math.cos(yaw)
                    yaw2   = yaw + dtheta
                    x      = cx + r_min * seg.turn * math.sin(yaw2)
                    y      = cy - r_min * seg.turn * math.cos(yaw2)
                    yaw    = Geometry.wrap(yaw2)

        poses.append((x, y, yaw, poses[-1][3] if poses else F))
        return poses
