"""
bezier_hull.py — Bézier Convex Hull Safety Pre-Validator
=========================================================

MATHEMATICAL BACKGROUND
-----------------------
Every trajectory segment produced by Waypoint.connect_to(wp2) is a 5th-degree
(quintic) polynomial in time.  By the equivalence between the canonical
polynomial basis and the Bernstein basis, this polynomial is *identically equal*
to a degree-5 Bézier curve.  A degree-5 Bézier curve has exactly 6 control
points P₀ … P₅.

THE CONVEX HULL PROPERTY
------------------------
A fundamental theorem of Bézier curves states that the curve lies entirely
within the convex hull of its control points (Farin, 2002, ch. 6).
Consequence: if **all 6 control points are inside a convex region** (our safe
OBB), the **entire trajectory is guaranteed to stay inside that region**.

DERIVATION OF CONTROL POINTS FROM BOUNDARY CONDITIONS
------------------------------------------------------
Given wp1 (t₁, pos₁, vel₁, acel₁) and wp2 (t₂, pos₂, vel₂, acel₂) with Δt = t₂-t₁:

Matching the 0th, 1st and 2nd derivatives of the Bernstein polynomial at u=0
and u=1 with the kinematic boundary conditions yields:

    P₀ = pos₁
    P₁ = pos₁ + vel₁ · (Δt / 5)
    P₂ = pos₁ + vel₁ · (2Δt / 5) + acel₁ · (Δt² / 20)
    P₃ = pos₂ - vel₂ · (2Δt / 5) + acel₂ · (Δt² / 20)
    P₄ = pos₂ - vel₂ · (Δt / 5)
    P₅ = pos₂

Reference: Farin, G. (2002). *Curves and Surfaces for CAGD: A Practical Guide*
           (5th ed.). Morgan Kaufmann.

USAGE IN THE RESOLUTION PIPELINE
---------------------------------
This module is consumed exclusively by build_trapezoid_detour() in
path_geometry.py.  The check is a LOCAL geometric pre-filter:

    1. Bézier hull check  ← this module       (fast, local, mathematical)
    2. connect_waypoints()                     (commits the polynomial)
    3. R-Tree shadow validation                (global, catches secondary conflicts)

The hull check can only REJECT candidates quickly; it never replaces the
R-Tree validation.  Both are always executed.
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING, List, Tuple

from core.config import NORM_ZERO_THRESHOLD

if TYPE_CHECKING:
    from core.models.waypoint import Waypoint
    from detection.conflictDetection import SweptBox_OBB


# ---------------------------------------------------------------------------
# 1. Control Point Computation
# ---------------------------------------------------------------------------

def compute_bezier_control_points(
    wp1: "Waypoint",
    wp2: "Waypoint",
) -> List[np.ndarray]:
    """
    Compute the 6 Bézier control points P₀…P₅ for the degree-5 Bézier curve
    that is *mathematically equivalent* to the quintic polynomial produced by
    ``wp1.connect_to(wp2)``.

    Derivation
    ----------
    The quintic canonical polynomial is::

        r(τ) = pos₁ + vel₁·τ + (acel₁/2)·τ²
               + (jerk₁/6)·τ³ + (snap₁/24)·τ⁴ + (crkl₁/120)·τ⁵

    where τ = t - t₁.  Substituting τ = u·Δt (u ∈ [0,1]) and matching the
    0th, 1st and 2nd Bernstein derivatives at u=0 and u=1 with the boundary
    conditions gives the formula documented in the module docstring.

    Args:
        wp1: Start waypoint — must have .t, .pos, .vel, .acel set.
        wp2: End   waypoint — must have .t, .pos, .vel, .acel set.

    Returns:
        List of 6 numpy arrays [P₀, P₁, P₂, P₃, P₄, P₅].
    """
    dt = float(wp2.t - wp1.t)

    if dt <= 0.0:
        # Degenerate time interval — collapse all points to wp1.pos
        return [wp1.pos.copy() for _ in range(6)]

    dt2 = dt * dt

    P0 = wp1.pos.copy()
    P1 = wp1.pos + wp1.vel  * (dt  / 5.0)
    P2 = wp1.pos + wp1.vel  * (2.0 * dt / 5.0) + wp1.acel * (dt2 / 20.0)
    P3 = wp2.pos - wp2.vel  * (2.0 * dt / 5.0) + wp2.acel * (dt2 / 20.0)
    P4 = wp2.pos - wp2.vel  * (dt  / 5.0)
    P5 = wp2.pos.copy()

    return [P0, P1, P2, P3, P4, P5]


# ---------------------------------------------------------------------------
# 2. Per-Segment Convex Hull Check
# ---------------------------------------------------------------------------

def all_control_points_inside_obb(
    wp1: "Waypoint",
    wp2: "Waypoint",
    obb: "SweptBox_OBB",
) -> bool:
    """
    Check whether all 6 Bézier control points for the segment [wp1 → wp2]
    lie inside *obb*.

    If this returns True, the Convex Hull Property guarantees that the entire
    trajectory of the segment is contained within ``obb``.

    Args:
        wp1: Start waypoint.
        wp2: End   waypoint.
        obb: The OBB to test containment against (uses ``obb.contains_point``).

    Returns:
        True only if every one of the 6 control points is inside the OBB.
    """
    points = compute_bezier_control_points(wp1, wp2)
    return all(obb.contains_point(p) for p in points)


# ---------------------------------------------------------------------------
# 3. Trapezoid Hull Check (flat segment only)
# ---------------------------------------------------------------------------

def check_trapezoid_flat_hull(
    wp_det1: "Waypoint",
    wp_det2: "Waypoint",
    obb_safe: "SweptBox_OBB",
) -> Tuple[bool, List[np.ndarray]]:
    """
    Check the Bézier Convex Hull Property for the **flat top** segment of the
    trapezoid (det1 → det2).

    This is the only segment that must be verified against the safe OBB.
    The ramp segments (anc→det1 and det2→ret) are implicitly safe because
    they occur in a different time window from the conflict.

    Args:
        wp_det1:  Detour entry waypoint (start of flat top).
        wp_det2:  Detour exit  waypoint (end   of flat top).
        obb_safe: The safe OBB — the collision OBB displaced by the MTV.

    Returns:
        (hull_ok, control_points) where *hull_ok* is True if all 6 points are
        inside obb_safe, and *control_points* is the list [P₀…P₅] for
        debugging / logging.
    """
    points = compute_bezier_control_points(wp_det1, wp_det2)
    hull_ok = all(obb_safe.contains_point(p) for p in points)
    return hull_ok, points


# ---------------------------------------------------------------------------
# 4. Safe OBB Construction
# ---------------------------------------------------------------------------

def build_safe_evasion_obb(
    conflict_obb: "SweptBox_OBB",
    mtv: np.ndarray,
) -> "SweptBox_OBB":
    """
    Build the "safe OBB" by translating the conflicting OBB's centre by the MTV.

    The resulting OBB represents the airspace corridor that is guaranteed to be
    spatially separated from the conflicting UAV by at least the MTV distance
    (which already includes SAFETY_MTV_SCALE margin from sat_mtv.py).

    Geometrically:
        safe_obb.center     = conflict_obb.center + mtv
        safe_obb.axes       = conflict_obb.axes       (same orientation)
        safe_obb.half_extents = conflict_obb.half_extents  (same size)
        safe_obb.t_range    = conflict_obb.t_range    (same time window)

    The flat top of the trapezoid (det1 → det2) is routed precisely through
    this safe OBB, parallel to the original route but displaced sideways or
    vertically by the MTV.

    Since ``conflict_obb`` was generated by ``generate_swept_boxes_obb`` its
    axis[0] is aligned with the UAV's forward direction.  Therefore:
        det1 = safe_obb.center - half_extents[0] · axes[0]   (entry corner)
        det2 = safe_obb.center + half_extents[0] · axes[0]   (exit  corner)

    Args:
        conflict_obb: The SweptBox_OBB that was flagged as colliding.
        mtv:          The Minimum Translation Vector (already SAFETY_MTV_SCALE-scaled).

    Returns:
        A new SweptBox_OBB with centre shifted by mtv; all other fields identical.
    """
    from detection.conflictDetection import SweptBox_OBB

    new_center = conflict_obb.center + np.array(mtv, dtype=float)

    return SweptBox_OBB(
        center       = new_center,
        axes         = conflict_obb.axes.copy(),
        half_extents = conflict_obb.half_extents.copy(),
        t_start      = conflict_obb.t_range[0],
        t_end        = conflict_obb.t_range[1],
    )
