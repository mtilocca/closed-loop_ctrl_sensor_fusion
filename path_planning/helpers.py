"""
helpers.py — Vehicle configuration for the path_planning package.
"""

from __future__ import annotations

import math


class VehicleConfig:
    """
    XCMG XDE360 (or any car-like vehicle) configuration.

    All planner classes accept a VehicleConfig instance so that vehicle
    parameters are explicit, not hidden in module-level globals.

    Example::

        vehicle = VehicleConfig(wheelbase_m=5.5, max_steer_deg=15.0)
        print(vehicle.r_min)   # → 20.53 m
    """

    def __init__(
        self,
        wheelbase_m:     float = 5.5,
        max_steer_deg:   float = 15.0,   # → r_min ≈ 20.53 m
        cruise_fwd_mps:  float = 5.5,
        cruise_rev_mps:  float = 2.5,
        arrive_radius_m: float = 2.0,
        cusp_speed_mps:  float = 0.0,
    ) -> None:
        self.wheelbase_m     = wheelbase_m
        self.max_steer_deg   = max_steer_deg
        self.cruise_fwd_mps  = cruise_fwd_mps
        self.cruise_rev_mps  = cruise_rev_mps
        self.arrive_radius_m = arrive_radius_m
        self.cusp_speed_mps  = cusp_speed_mps

    @property
    def r_min(self) -> float:
        """Minimum turning radius derived from wheelbase and max steering angle."""
        return self.wheelbase_m / math.tan(math.radians(self.max_steer_deg))

    def __repr__(self) -> str:
        return (
            f"VehicleConfig(wheelbase={self.wheelbase_m}m, "
            f"max_steer={self.max_steer_deg}°, r_min={self.r_min:.2f}m, "
            f"fwd={self.cruise_fwd_mps}m/s, rev={self.cruise_rev_mps}m/s)"
        )
