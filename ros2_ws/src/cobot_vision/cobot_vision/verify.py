"""Post-place depth verdict for /vision/verify_target — pure, no ROS.

The task manager calls the service after a pick has been placed (the arm
covers the camera mid-pick). localization_node re-samples the depth at the
pixel the target was detected at and compares it with the depth measured
there at scan time: a can that was taken leaves the surface roughly one can
height deeper, i.e. a LARGER depth reading.
"""

from __future__ import annotations


def classify_presence(
    expected_mm: float, measured_mm: float, tolerance_mm: float
) -> tuple[bool, float]:
    """(present, delta_mm). delta = measured - expected; positive means the
    surface is now deeper than the detected top, so the object is gone."""
    delta = float(measured_mm) - float(expected_mm)
    return abs(delta) <= float(tolerance_mm), delta
