"""
clothoid.py — Full clothoid arc (double Euler spiral) sampler.

Replaces the RS circular arc with a G2-continuous curvature profile:
    κ(s): 0 → 1/r_min (at midpoint) → 0

For a 220-ton mining truck the steering actuator has a finite slew rate;
instantaneous curvature steps (RS circular arcs) cause steering overshoot
and lateral path deviation. Clothoid arcs eliminate this problem.
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


class ClothoidSampler:
    """
    Samples a full clothoid arc (double Euler spiral) for a given RS arc segment.

    Usage::

        poses = ClothoidSampler.sample(
            x0, y0, yaw0,
            seg_length=arc_length_m,
            turn_sign=+1,   # +1 left, -1 right
            dir_sign=+1,    # +1 forward, -1 reverse
            r_min=20.53,
            step=0.5,
        )
    """

    @staticmethod
    def sample(
        x0:         float,
        y0:         float,
        yaw0:       float,
        seg_length: float,
        turn_sign:  float,
        dir_sign:   int,
        r_min:      float,
        step:       float = 0.5,
    ) -> List[Tuple[float, float, float, int]]:
        """
        Sample a full clothoid arc (double Euler spiral).

        The curvature profile is triangular: κ ramps linearly from 0 to 1/r_min
        at the arc midpoint, then back to 0.  This guarantees G2-continuous
        (curvature-continuous) steering — no instantaneous steering jumps at
        arc/straight junctions.

        The clothoid achieves the **same turning angle** Δθ as the RS circular
        arc it replaces, but the path length is **2× longer** (κ_max = 1/r_min).

        Args:
            x0, y0, yaw0: Starting pose in path-local frame (metres, radians).
            seg_length:   Length of the *original* RS circular arc (metres).
            turn_sign:    +1 = left turn, −1 = right turn.
            dir_sign:     +1 = forward, −1 = reverse.
            r_min:        Minimum turning radius (metres).
            step:         Sample interval in metres.

        Returns:
            List of (x, y, yaw_rad, dir) poses starting at (x0, y0, yaw0).

        Math note:
            RS arc: arc_length = r_min × Δθ
            Clothoid: L = 2 × arc_length,  κ_max = 1/r_min
            ∫₀^L κ(s)ds = κ_max × L/2 = L/(2r_min) = arc_length/r_min = Δθ  ✓
        """
        L = 2.0 * seg_length      # clothoid arc = 2 × RS circular arc length
        if L < 1e-9:
            return [(x0, y0, yaw0, dir_sign)]

        half = L / 2.0
        n    = max(4, int(L / step) + 1)
        s_arr = np.linspace(0.0, L, n)

        def _dtheta(s: float) -> float:
            """Analytical heading change at arc-length position s."""
            if s <= half:
                return s * s / (2.0 * half * r_min)
            return (half / (2.0 * r_min)
                    + (s - half) / r_min
                    - (s - half) ** 2 / (2.0 * half * r_min))

        dtheta = np.array([_dtheta(float(sv)) for sv in s_arr])
        theta  = yaw0 + dir_sign * turn_sign * dtheta   # heading at each sample

        # Numerical position integration (forward Euler, left-endpoint rectangles)
        ds     = L / (n - 1)
        x_arr  = np.empty(n)
        y_arr  = np.empty(n)
        x_arr[0] = x0
        y_arr[0] = y0
        x_arr[1:] = x0 + dir_sign * np.cumsum(np.cos(theta[:-1])) * ds
        y_arr[1:] = y0 + dir_sign * np.cumsum(np.sin(theta[:-1])) * ds

        return list(zip(x_arr.tolist(), y_arr.tolist(),
                        theta.tolist(), [dir_sign] * n))
