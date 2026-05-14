"""
sat_mtv.py — SAT All-Axes MTV Extractor
=========================================

PURPOSE:
    Given a confirmed collision between two OBBs (from the conflict dict),
    extract the MTV (Minimum Translation Vector) for EACH of the 15 SAT axes
    individually. This gives us up to 15 displacement candidates, one per axis.

    This is fundamentally different from collides_with(), which only tracks and
    returns the MINIMUM overlap across all axes. Here we collect ALL axis overlaps
    so the resolver can try them in ascending order of disruption.

ALGORITHM:
    Mirrors the axis-testing loop inside SweptBox_OBB.collides_with(), but instead
    of tracking only the minimum, records the overlap MTV for every non-degenerate
    axis. Since we are called only after a confirmed collision (all axes overlap),
    no axis should return a separation — any that do are skipped as numerical noise.

OUTPUT:
    SATResult with two sorted lists ready for the cascade:
        - horizontal_mtvs: axes where XY component dominates, ascending magnitude
        - vertical_mtvs:   axes where Z component dominates, ascending magnitude

SAFETY MARGIN:
    Each raw MTV is multiplied by SAFETY_MTV_SCALE (default 1.05) to ensure a
    small clearance margin beyond the bare minimum separation distance. This
    prevents borderline cases where the resolved plan still barely touches the
    R-Tree box of the VIP.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List

from core.config import (
    EPSILON_ABSOLUTE,
    EPSILON_ORTHOGONAL,
    SAFETY_MTV_SCALE,
)

if TYPE_CHECKING:
    from detection.rtree_detector import RTreeDetector


# ---------------------------------------------------------------------------
# Data Structures
# ---------------------------------------------------------------------------

@dataclass
class SATResult:
    """
    Output of generate_mtv_candidates().

    Attributes:
        horizontal_mtvs: MTV vectors where XY dominates, sorted ascending magnitude.
                         Used by Strategy 2 (Path Stretch).
        vertical_mtvs:   MTV vectors where Z dominates, sorted ascending magnitude.
                         Used by Fallback 1 (Vertical MTVs).
        success:         False if the OBBs could not be retrieved or no overlap found.
    """
    horizontal_mtvs: List[np.ndarray] = field(default_factory=list)
    vertical_mtvs:   List[np.ndarray] = field(default_factory=list)
    success:         bool              = True


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def generate_mtv_candidates(
    conflict: dict,
    manager:  "RTreeDetector",
    flight_plan_pleb=None,
    min_perpendicularity: float = 0.3,
    t_ref: float | None = None,
) -> SATResult:
    """
    Extract all per-axis MTVs from the SAT computation for a confirmed conflict.

    The conflict dict is the format returned by RTreeDetector.detect_all_conflicts():
        {
            "uav_a":      plebeian_id,
            "uav_b":      vip_id,
            "box_a_idx":  int,   ← index into manager.uavs[uav_a]["boxes"]
            "box_b_idx":  int,
            "time_range": (t_start, t_end),
            "mtv":        np.ndarray  ← minimum MTV (from collides_with — not used here)
        }

    We retrieve the actual OBB objects and re-run the 15-axis loop ourselves,
    collecting the MTV for every axis (not just the minimum).

    Args:
        conflict: Conflict dict from detect_all_conflicts().
        manager:  Active RTreeDetector holding all registered OBB boxes.
        flight_plan_pleb: Optional FlightPlan of plebeian UAV to filter MTVs by 
                         perpendicularity to instantaneous velocity at conflict time.
        min_perpendicularity: Minimum perpendicularity threshold (|dot product| < this value).
                             MTVs with |dot(mtv, velocity)| >= this are discarded.
                             Default: 0.3 (rejects MTVs more aligned than ~73° to flight direction).
        t_ref: Optional evaluation time for the velocity vector. If not provided, defaults to 
               the temporal midpoint of the conflict (matching the detour apex time).

    Returns:
        SATResult with ranked horizontal and vertical MTV lists.
    """
    pleb_id = conflict["uav_a"]
    vip_id  = conflict["uav_b"]

    # Determine evaluation time (apex time)
    if t_ref is not None:
        t_eval = t_ref
    else:
        # Default to the temporal midpoint of the conflict window, which
        # is mathematically equivalent to the initial apex time (t_det)
        # calculated in build_rigid_shift_detour.
        t_start, t_end = conflict["time_range"]
        t_eval = (t_start + t_end) / 2.0

    # =========================================================================
    # Step 1: Retrieve the OBB objects that collided
    # GEOMETRIC REFINEMENT: Attempt to retrieve the specific OBB pair active at
    # the evaluation time (apex) rather than blindly using conflict["box_a_idx"].
    # This guarantees that the resulting MTV is derived from the true geometry
    # at the apex of the conflict.
    # =========================================================================
    try:
        # Baseline fallback: original colliding boxes
        box_pleb = manager.uavs[pleb_id]["boxes"][conflict["box_a_idx"]]
        box_vip  = manager.uavs[vip_id]["boxes"][conflict["box_b_idx"]]
        
        # Dynamic lookup for the OBB pair active at t_eval (apex)
        best_box_pleb = None
        for box in manager.uavs[pleb_id]["boxes"]:
            if box.t_range[0] <= t_eval <= box.t_range[1]:
                best_box_pleb = box
                break
        
        if best_box_pleb is not None:
            for v_box in manager.uavs[vip_id]["boxes"]:
                # Temporal overlap check
                if not (v_box.t_range[1] <= best_box_pleb.t_range[0] or v_box.t_range[0] >= best_box_pleb.t_range[1]):
                    collides, _ = best_box_pleb.collides_with(v_box)
                    if collides:
                        box_pleb = best_box_pleb
                        box_vip = v_box
                        break
    except (KeyError, IndexError):
        return SATResult(success=False)

    # =========================================================================
    # Step 2: Build the 15 candidate axes (same as in collides_with)
    #   - 3 face-normal axes from the plebeian OBB
    #   - 3 face-normal axes from the VIP OBB
    #   - 9 edge-edge cross-product axes (3 × 3)
    # =========================================================================
    axes_to_test: List[np.ndarray] = []
    axes_to_test.extend(box_pleb.axes)   # 3 axes
    axes_to_test.extend(box_vip.axes)    # 3 axes

    for i in range(3):
        for j in range(3):
            cross = np.cross(box_pleb.axes[i], box_vip.axes[j])
            if np.linalg.norm(cross) > EPSILON_ABSOLUTE:
                axes_to_test.append(cross)   # up to 9 axes

    # =========================================================================
    # Step 3: For each axis, compute the overlap and build its MTV
    # Since collides_with() already confirmed a collision, every valid axis
    # should show a positive overlap. Skip degenerate or separating axes.
    # =========================================================================
    raw_mtvs: List[np.ndarray] = []

    for axis in axes_to_test:
        axis_len = np.linalg.norm(axis)
        if axis_len < EPSILON_ABSOLUTE:
            continue  # Degenerate axis — skip

        # Normalize (same logic as in collides_with / project_on_axis)
        if abs(axis_len - 1.0) > EPSILON_ORTHOGONAL:
            axis_norm = axis / axis_len
        else:
            axis_norm = axis

        # Project both OBBs onto this axis
        min1, max1 = box_pleb.project_on_axis(axis_norm)
        min2, max2 = box_vip.project_on_axis(axis_norm)

        # Overlap amount on this axis
        overlap = min(max1 - min2, max2 - min1)

        if overlap <= 0:
            # Axis separates the boxes — numerical noise from near-miss; skip
            continue

        # Orient the push direction so it moves the plebeian AWAY from the VIP
        if np.dot(box_pleb.center, axis_norm) < np.dot(box_vip.center, axis_norm):
            mtv = -axis_norm * overlap * SAFETY_MTV_SCALE
        else:
            mtv = axis_norm * overlap * SAFETY_MTV_SCALE

        raw_mtvs.append(mtv)

    if not raw_mtvs:
        # Edge case: no valid axis found (shouldn't happen after confirmed collision)
        return SATResult(success=False)

    # =========================================================================
    # Step 4a: FILTER MTVs BY PERPENDICULARITY TO INSTANTANEOUS VELOCITY
    # An MTV parallel to the plebeian's current velocity is not a true evasion—
    # it just asks the drone to accelerate/decelerate along its current direction.
    # We filter to keep only MTVs that are sufficiently perpendicular to the
    # instantaneous velocity vector at the conflict evaluation time.
    # =========================================================================
    if flight_plan_pleb is not None:
        velocity_at_conflict = flight_plan_pleb.status_at_time(t_eval).vel
        
        if velocity_at_conflict is not None:
            velocity = np.array(velocity_at_conflict)
            vel_norm = np.linalg.norm(velocity)
            
            # Only filter if the UAV is actually moving at conflict time
            if vel_norm > EPSILON_ABSOLUTE:
                vel_direction = velocity / vel_norm
                
                # Keep only MTVs that are sufficiently perpendicular to velocity
                filtered_mtvs = []
                for mtv in raw_mtvs:
                    mtv_norm = mtv / np.linalg.norm(mtv)
                    alignment = abs(np.dot(mtv_norm, vel_direction))
                    
                    # Discard if too aligned with velocity (|dot| >= threshold)
                    if alignment < min_perpendicularity:
                        filtered_mtvs.append(mtv)
                
                # Use filtered list if non-empty, otherwise keep all
                if filtered_mtvs:
                    raw_mtvs = filtered_mtvs

    # =========================================================================
    # Step 4: Split into Horizontal (XY-dominant) and Vertical (Z-dominant)
    # An MTV is Vertical if |Z| >= max(|X|, |Y|), Horizontal otherwise.
    # Sort each list ascending by Euclidean magnitude (minimum disruption first).
    # =========================================================================
    horizontal_mtvs: List[np.ndarray] = []
    vertical_mtvs:   List[np.ndarray] = []

    for mtv in raw_mtvs:
        abs_z  = abs(mtv[2])
        abs_xy = max(abs(mtv[0]), abs(mtv[1]))

        if abs_z >= abs_xy:
            vertical_mtvs.append(mtv)
        else:
            horizontal_mtvs.append(mtv)

    horizontal_mtvs.sort(key=lambda v: float(np.linalg.norm(v)))
    vertical_mtvs.sort(key=lambda v: float(np.linalg.norm(v)))

    return SATResult(
        horizontal_mtvs=horizontal_mtvs,
        vertical_mtvs=vertical_mtvs,
        success=True,
    )
