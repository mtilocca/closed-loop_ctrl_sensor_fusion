"""
geometry.py — Pure geometry utility class for 2-D path planning.
"""

from __future__ import annotations

import math
from typing import Tuple


class Geometry:
    """Static geometry helpers used throughout the path_planning package."""

    @staticmethod
    def wrap(a: float) -> float:
        """Wrap angle to [-π, π]."""
        return (a + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def mod2pi(a: float) -> float:
        """Wrap angle to [0, 2π)."""
        return a % (2 * math.pi)

    @staticmethod
    def polar(x: float, y: float) -> Tuple[float, float]:
        """Cartesian → polar (r, θ)."""
        return math.hypot(x, y), math.atan2(y, x)

    @staticmethod
    def parse_pose(s: str) -> Tuple[float, float, float]:
        """Parse 'x,y,yaw_deg' string → (x_m, y_m, yaw_deg) floats."""
        parts = [float(v.strip()) for v in s.split(",")]
        if len(parts) != 3:
            raise ValueError(f"Expected 'x,y,yaw_deg', got: {s!r}")
        return parts[0], parts[1], parts[2]
