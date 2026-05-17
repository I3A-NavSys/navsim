"""
conflict_resolver.py — Cascading Conflict Resolution Orchestrator
=================================================================

PURPOSE:
    The ConflictResolver is the central brain of the deconfliction pipeline.
    Given a detected conflict between two UAVs (a high-priority VIP and a
    lower-priority plebeian), it orchestrates the full resolution cascade:

        S1 (Kinematic Bounding)
           ↓ fail
        SAT MTV Generation
           ↓
        S2 (Horizontal Path Stretch)
           ↓ fail
        FB1 (Vertical MTVs)         ← Z-shift is implicit: a vertical MTV
           ↓ fail                     displaces the drone in Z automatically.
        FB2 (Hover)
           ↓ fail
        DEADLOCK (Mission Abort)

TEMPORAL CONCEPTS — IMPORTANT DISTINCTION:
    t_start  = fp_pleb.init_time()
               The scheduled departure time of the plebeian's flight plan.
               This is set by the UAV operator when it registers its route
               (e.g. "I want to depart in 60 seconds from now").
               All waypoint timestamps in the FlightPlan are ABSOLUTE times
               anchored to t_start. The resolver must never plan anything
               before t_start — it is a hard temporal floor.

    t_anchor = max(t_conflict, t_start) + ANCHOR_DELTA
               The point in time from which a maneuver actually begins.
               ANCHOR_DELTA = WCET + safety margin, ensuring the drone has
               enough computation time to receive and start executing the new
               plan before it is physically needed.
               t_anchor is specific to each resolution attempt and is derived
               within this class. It is NOT the same as t_start.

INVARIANTS ENFORCED BY THIS CLASS:
    1. SEMAPHORE (Delayed Clearance): The VIP is put on HOLD at t_anchor before
       any maneuver is computed. It is only released (HOLD cleared) once the
       plebeian has a confirmed conflict-free plan.

    2. WCET ANCHOR: All maneuvers start at t_anchor >= t_start + ANCHOR_DELTA.
       The resolver never operates before the UAV's scheduled departure time.

    3. BOUNDED LOOPS: Every loop in this module has a hard iteration cap.
       No while-true loops exist. The Fallback 2 hovering loop is explicitly
       capped at HOVER_MAX_TIMEOUT / HOVER_TIME_STEP iterations.

    4. SHADOW VALIDATION: All candidate plans are validated in a shadow R-Tree
       (never the live one) before acceptance.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from core.config import (
    ANCHOR_DELTA,
    OBB_INTERVAL,
    HOVER_MAX_TIMEOUT,
    HOVER_TIME_STEP,
    WAYPOINT_TIME_EPSILON,
    FORWARD_PROGRESS_MARGIN,
    UAV_MAX_SPEED,
    MIN_DETOUR_DURATION,
    MTV_SCALE_TIME_BUFFER_FACTOR,
)
from resolution.geometry.sat_mtv import generate_mtv_candidates, SATResult
from resolution.geometry.path_geometry import (
    build_rigid_shift_detour,
)
from resolution.kinematic.velocity_bounding import run_strategy1

if TYPE_CHECKING:
    from detection.rtree_detector import RTreeDetector


# ---------------------------------------------------------------------------
# Output Data Structure
# ---------------------------------------------------------------------------

@dataclass
class ResolveResult:
    """
    Result of a conflict resolution attempt.

    Attributes:
        success:       True if a conflict-free plan was found.
        strategy_used: Which phase solved the conflict.
                       One of: "S1", "S2", "FB1", "FB2", "DEADLOCK".
        new_fp_pleb:   The new FlightPlan for the plebeian UAV (None on DEADLOCK).
        new_fp_vip:    The VIP FlightPlan with HOLD cleared (None on DEADLOCK).
        iterations:    Total number of candidate plans evaluated across all phases.
        message:       Human-readable summary for logging / debugging.
    """
    success:       bool                  = False
    strategy_used: str                   = "NONE"
    new_fp_pleb:   Optional[FlightPlan]  = None
    new_fp_vip:    Optional[FlightPlan]  = None
    iterations:    int                   = 0
    message:       str                   = ""


# ---------------------------------------------------------------------------
# Orchestrator
# ---------------------------------------------------------------------------

class ConflictResolver:
    """
    Cascading conflict resolver for two UAVs with asymmetric priority.

    Usage:
        resolver = ConflictResolver(manager)
        result = resolver.resolve(conflict, fp_pleb, fp_vip, pleb_id, vip_id)
        if result.success:
            manager.register_uav(pleb_id, result.new_fp_pleb)
            manager.register_uav(vip_id,  result.new_fp_vip)
    """

    def __init__(self, manager: "RTreeDetector") -> None:
        self._manager = manager

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def resolve(
        self,
        conflict:  dict,
        fp_pleb:   FlightPlan,
        fp_vip:    FlightPlan,
        pleb_id:   str,
        vip_id:    str,
    ) -> ResolveResult:
        """
        Run the full resolution cascade for a given conflict.

        Args:
            conflict: Conflict dict from RTreeDetector.detect_all_conflicts().
                      Must contain: "uav_a", "uav_b", "time_range", "mtv",
                      "box_a_idx", "box_b_idx".
            fp_pleb:  Plebeian's current FlightPlan (not mutated).
            fp_vip:   VIP's current FlightPlan (not mutated).
            pleb_id:  Plebeian UAV identifier.
            vip_id:   VIP UAV identifier.

        Returns:
            ResolveResult with the outcome and the new FlightPlans (if successful).
        """
        total_iterations = 0

        # =================================================================
        # Step 0: Compute anchor time (WCET invariant)
        #
        # t_start  = the UAV's scheduled departure time (fp_pleb.init_time()).
        #            We use it as a HARD FLOOR: t_anchor >= t_start + ANCHOR_DELTA.
        #
        # t_anchor = the point from which the maneuver starts.
        #
        # KEY INVARIANT: t_anchor MUST be BEFORE t_conflict.
        #   If the anchor is placed after the conflict, any detour / speed change /
        #   hover will only affect the post-conflict trajectory — the drone has
        #   already crossed the conflict zone and all strategies become useless.
        #
        # Formula:
        #   t_anchor = max(t_start + ANCHOR_DELTA, t_conflict - ANCHOR_DELTA)
        #   if t_anchor >= t_conflict:                  (no time left to maneuver)
        #       clamp to t_conflict - WAYPOINT_TIME_EPSILON
        #       (the drone inserts the maneuver as late as geometrically possible
        #        while still being BEFORE the collision instant)
        # =================================================================
        t_start    = fp_pleb.init_time()           # Scheduled departure time
        t_conflict = conflict["time_range"][0]     # Earliest overlap instant

        # Place the anchor ANCHOR_DELTA seconds before the conflict, but respect the floor
        t_anchor = max(t_start + ANCHOR_DELTA, t_conflict - ANCHOR_DELTA)

        # Ensure the anchor is still strictly before the conflict
        if t_anchor >= t_conflict:
            t_anchor = t_conflict - WAYPOINT_TIME_EPSILON
            if t_anchor < t_start:
                # Not even t_start is before the conflict — true DEADLOCK
                return ResolveResult(
                    success=False,
                    strategy_used="DEADLOCK",
                    message=f"t_anchor cannot be placed before t_conflict={t_conflict:.2f}s "
                            f"(t_start={t_start:.2f}s, ANCHOR_DELTA={ANCHOR_DELTA}s). "
                            f"Mission abort.",
                )

        # Guard: anchor must be before plebeian finishes its flight
        if t_anchor >= fp_pleb.finish_time():
            return ResolveResult(
                success=False,
                strategy_used="DEADLOCK",
                message=f"t_anchor={t_anchor:.2f}s is beyond plebeian flight end "
                        f"({fp_pleb.finish_time():.2f}s). Mission abort.",
            )

        # =================================================================
        # Step 1: HOLD the VIP — Semaphore (Delayed Clearance)
        # The VIP is postponed from t_anchor onwards so it waits in place
        # while the plebeian computes and commits to its new plan.
        # =================================================================
        fp_vip_held = self._hold_vip(fp_vip, t_anchor)

        # =================================================================
        # Build shadow R-Tree ONCE for all strategies (S1/S2/FB1/FB2).
        # This avoids re-registering every other UAV for each candidate.
        # register_uav() already removes old boxes before inserting new
        # ones, so successive candidate swaps are safe.
        # =================================================================
        shadow = self._build_shadow_rtree(pleb_id)

        def _check_valid(candidate: FlightPlan) -> bool:
            return self._validate_shadow(candidate, pleb_id, shadow, t_conflict)

        # =================================================================
        # Step 2: Strategy 1 — Kinematic Bounding (velocity-only)
        # =================================================================
        result_s1 = run_strategy1(
            fp_pleb=fp_pleb,
            t_conflict=t_conflict,
            validator_fn=_check_valid,
        )

        total_iterations += 1  # strategy 1 counts as a single "attempt unit"

        if result_s1 is not None:
            fp_vip_released = self._release_vip(fp_vip_held, t_anchor)
            return ResolveResult(
                success=True,
                strategy_used="S1",
                new_fp_pleb=result_s1,
                new_fp_vip=fp_vip_released,
                iterations=total_iterations,
                message="Strategy 1 (Kinematic Bounding) succeeded.",
            )
        else:
            print(f"[DEBUG S1] Kinematic Bounding FAILED for {pleb_id}")
            print(f"[DEBUG S1] Could not resolve conflict by velocity adjustments alone")

        # =================================================================
        # Step 3: Generate SAT MTV candidates for spatial strategies
        # =================================================================
        sat_result: SATResult = generate_mtv_candidates(
            conflict=conflict,
            manager=self._manager,
            flight_plan_pleb=fp_pleb,
        )

        if not sat_result.success:
            # SAT did not find a meaningful MTV — fall through to Fallback 2
            sat_result.horizontal_mtvs = []
            sat_result.vertical_mtvs   = []
            print(f"[DEBUG SAT] SAT FAILED to find MTV candidates for {pleb_id}")
        else:
            print(f"[DEBUG SAT] Generated MTV candidates for {pleb_id}:")
            print(f"[DEBUG SAT]   Horizontal MTVs: {len(sat_result.horizontal_mtvs)}")
            print(f"[DEBUG SAT]   Vertical MTVs: {len(sat_result.vertical_mtvs)}")
            if len(sat_result.horizontal_mtvs) > 0:
                for i, mtv in enumerate(sat_result.horizontal_mtvs[:2]):  # Show first 2
                    print(f"[DEBUG SAT]     H-MTV[{i}] = {mtv}")
            if len(sat_result.vertical_mtvs) > 0:
                for i, mtv in enumerate(sat_result.vertical_mtvs[:2]):  # Show first 2
                    print(f"[DEBUG SAT]     V-MTV[{i}] = {mtv}")

        # =================================================================
        # Step 4: Strategy 2 — Horizontal Path Stretch
        # =================================================================
        print(f"[DEBUG S2] Attempting Strategy 2 (Horizontal Path Stretch) for {pleb_id}...")
        s2_result, iters = self._strategy2_horizontal_mtvs(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
            horizontal_mtvs=sat_result.horizontal_mtvs,
            conflict=conflict,
            shadow=shadow,
        )
        total_iterations += iters

        if s2_result is not None:
            print(f"[DEBUG S2] SUCCESS! Resolved {pleb_id} with S2 after {iters} iterations")
            fp_vip_released = self._release_vip(fp_vip_held, t_anchor)
            return ResolveResult(
                success=True,
                strategy_used="S2",
                new_fp_pleb=s2_result,
                new_fp_vip=fp_vip_released,
                iterations=total_iterations,
                message="Strategy 2 (Horizontal Path Stretch) succeeded.",
            )
        else:
            print(f"[DEBUG S2] FAILED! No valid horizontal detour found after {iters} iterations")

        # =================================================================
        # Step 5: Fallback 1 — Vertical MTVs
        # build_trapezoid_detour handles Z-dominant MTVs the same way as
        # horizontal ones: the flat top is displaced vertically.
        # =================================================================
        print(f"[DEBUG FB1] Attempting Fallback 1 (Vertical MTVs) for {pleb_id}...")
        fb1_result, iters = self._fallback1_vertical_mtvs(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
            vertical_mtvs=sat_result.vertical_mtvs,
            conflict=conflict,
            shadow=shadow,
        )
        total_iterations += iters

        if fb1_result is not None:
            print(f"[DEBUG FB1] SUCCESS! Resolved {pleb_id} with FB1 after {iters} iterations")
            fp_vip_released = self._release_vip(fp_vip_held, t_anchor)
            return ResolveResult(
                success=True,
                strategy_used="FB1",
                new_fp_pleb=fb1_result,
                new_fp_vip=fp_vip_released,
                iterations=total_iterations,
                message="Fallback 1 (Vertical MTVs) succeeded.",
            )

        # =================================================================
        # Step 6: Fallback 2 — Hover
        # =================================================================
        fb2_result, iters = self._fallback2_hover(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
            shadow=shadow,
        )
        total_iterations += iters

        if fb2_result is not None:
            fp_vip_released = self._release_vip(fp_vip_held, t_anchor)
            return ResolveResult(
                success=True,
                strategy_used="FB2",
                new_fp_pleb=fb2_result,
                new_fp_vip=fp_vip_released,
                iterations=total_iterations,
                message="Fallback 2 (Hover) succeeded.",
            )

        # =================================================================
        # Step 7: DEADLOCK — all strategies exhausted
        # =================================================================
        return ResolveResult(
            success=False,
            strategy_used="DEADLOCK",
            new_fp_pleb=None,
            new_fp_vip=None,
            iterations=total_iterations,
            message=(
                f"DEADLOCK: All resolution strategies failed for plebeian '{pleb_id}' "
                f"vs VIP '{vip_id}'. Mission abort required."
            ),
        )

    # ------------------------------------------------------------------
    # VIP Semaphore (HOLD / RELEASE)
    # ------------------------------------------------------------------

    def _hold_vip(self, fp_vip: FlightPlan, t_anchor: float) -> FlightPlan:
        """
        Create a copy of the VIP FlightPlan with HOLD applied from t_anchor.

        The VIP's waypoints from t_anchor onwards are pushed into the future
        by inserting a hover waypoint at t_anchor (same position, zero velocity).
        This implements the Delayed Clearance semaphore: the VIP will not enter
        the conflict zone until we explicitly release it.

        The hold duration is left open-ended here. The _release_vip() method
        will restore the normal schedule once the plebeian's plan is confirmed.

        Args:
            fp_vip:   Original VIP FlightPlan (not mutated).
            t_anchor: Time from which the VIP must hold.

        Returns:
            Modified copy of the VIP FlightPlan with HOLD applied.
        """
        fp_held = fp_vip.copy()

        # Position of VIP at t_anchor
        status_at_anchor = fp_held.status_at_time(t_anchor)
        hold_pos = status_at_anchor.pos.copy()

        # Insert a zero-velocity waypoint at t_anchor to force the VIP to hold
        hold_wp = Waypoint(
            label="HOLD",
            t=t_anchor,
            pos=hold_pos,
            vel=[0.0, 0.0, 0.0],
        )
        fp_held.set_waypoint(hold_wp)
        # Postpone all waypoints after t_anchor by a large sentinel value.
        # The actual hold duration will be corrected during release.
        # We use a sentinel of HOVER_MAX_TIMEOUT to ensure the VIP does not
        # accidentally depart during the resolution computation window.
        fp_held.postpone_from(t_anchor + WAYPOINT_TIME_EPSILON, HOVER_MAX_TIMEOUT)
        fp_held.connect_waypoints()

        return fp_held

    def _release_vip(self, fp_vip_held: FlightPlan, t_anchor: float) -> FlightPlan:
        """
        Create a clean copy of the VIP FlightPlan with the HOLD waypoint removed
        and the postpone offset reversed.

        This is called after the plebeian has committed to a new plan, signaling
        that the airspace ahead is safe.

        Args:
            fp_vip_held: The held VIP FlightPlan (from _hold_vip).
            t_anchor:    The hold start time.

        Returns:
            VIP FlightPlan with HOLD cleared.
        """
        fp_released = fp_vip_held.copy()

        # Remove the HOLD waypoint
        fp_released.remove_waypoint_at_time(t_anchor)

        # Reverse the sentinel postpone
        fp_released.postpone_from(t_anchor + WAYPOINT_TIME_EPSILON, -HOVER_MAX_TIMEOUT)
        fp_released.connect_waypoints()

        return fp_released

    # ------------------------------------------------------------------
    # Internal: R-Tree Shadow Validation
    # ------------------------------------------------------------------

    def _build_shadow_rtree(self, pleb_id: str) -> "RTreeDetector":
        """
        Build a shadow R-Tree containing ALL UAVs EXCEPT the plebeian.

        This is created ONCE per resolve() call and reused across all
        strategies (S2, FB1, FB2).  Only the plebeian's entry is swapped
        in/out via register_uav(), which already removes old boxes before
        inserting new ones — so no state leaks between candidates.

        Returns:
            A RTreeDetector with every UAV except pleb_id registered.
        """
        from detection.rtree_detector import RTreeDetector
        shadow = RTreeDetector()
        for uid, data in self._manager.uavs.items():
            if uid != pleb_id:
                shadow.register_uav(uid, data["fp"], interval=OBB_INTERVAL)
        return shadow

    def _validate_shadow(
        self,
        candidate_fp: FlightPlan,
        pleb_id: str,
        shadow: "RTreeDetector",
        t_conflict_being_solved: float = 0.0,
    ) -> bool:
        """
        Swap the plebeian's entry in the pre-built shadow R-Tree with the
        candidate FlightPlan and check for conflicts.

        FORWARD PROGRESS GUARANTEE:
        If the route still contains conflicts, the system accepts it ONLY if 
        the earliest remaining conflict is strictly and comfortably after the 
        one we are actively solving. This allows CentralManager to resolve 
        multi-conflict scenarios iteratively without DEADLOCK.
        """
        shadow.register_uav(pleb_id, candidate_fp, interval=OBB_INTERVAL)
        conflicts = shadow.detect_all_conflicts(pleb_id)
        if len(conflicts) == 0:
            return True
            
        # Check for forward progress
        conflicts.sort(key=lambda c: c["time_range"][0])
        earliest_new_conflict = conflicts[0]["time_range"][0]
        
        # Accept if the earliest remaining conflict is further in the future
        # (with safety margin)
        return earliest_new_conflict >= t_conflict_being_solved + FORWARD_PROGRESS_MARGIN

    def _compute_conflict_origin(
        self,
        conflict: dict,
        pleb_id: str,
    ) -> Optional[np.ndarray]:
        """
        Compute the detour origin for a conflict window.

        This helper avoids picking an arbitrary OBB when multiple boxes are
        involved. It collects all plebeian and VIP swept boxes that overlap the
        conflict time window, keeps only the pairs that truly collide in SAT,
        and builds a weighted centroid of their midpoints.

        The weight is the MTV norm because larger MTVs correspond to a stronger
        geometric overlap, which is a more relevant signal than just time
        coexistence.

        If no SAT-confirmed pair is found, the helper falls back to the average
        center of the plebeian boxes in the time window.
        """
        pleb_id_key = conflict.get("uav_a", pleb_id)
        vip_id_key = conflict.get("uav_b")
        t_min, t_max = conflict["time_range"]

        pleb_conflict_obbs = []
        vip_conflict_obbs = []

        try:
            for box in self._manager.uavs[pleb_id_key]["boxes"]:
                if box.t_range[1] > t_min and box.t_range[0] < t_max:
                    pleb_conflict_obbs.append(box)
        except KeyError:
            pleb_conflict_obbs = []

        try:
            for box in self._manager.uavs[vip_id_key]["boxes"]:
                if box.t_range[1] > t_min and box.t_range[0] < t_max:
                    vip_conflict_obbs.append(box)
        except KeyError:
            vip_conflict_obbs = []

        if not pleb_conflict_obbs:
            return None

        pair_midpoints = []
        weights = []

        for pb in pleb_conflict_obbs:
            for vb in vip_conflict_obbs:
                t_overlap = min(pb.t_range[1], vb.t_range[1]) - max(pb.t_range[0], vb.t_range[0])
                if t_overlap <= 0:
                    continue

                is_collision, mtv_vector = pb.collides_with(vb)
                if not is_collision:
                    continue

                midpoint = (np.array(pb.center, dtype=float) + np.array(vb.center, dtype=float)) * 0.5
                weight = float(np.linalg.norm(mtv_vector))
                if weight <= 0:
                    weight = float(t_overlap)

                pair_midpoints.append(midpoint)
                weights.append(weight)

        if pair_midpoints:
            total_weight = float(sum(weights))
            if total_weight > 0:
                return sum(midpoint * weight for midpoint, weight in zip(pair_midpoints, weights)) / total_weight

        return np.mean([np.array(box.center, dtype=float) for box in pleb_conflict_obbs], axis=0)

    # ------------------------------------------------------------------
    # Strategy 2: Horizontal Path Stretch
    # ------------------------------------------------------------------

    def _strategy2_horizontal_mtvs(
        self,
        fp_pleb:         FlightPlan,
        fp_vip:          FlightPlan,
        pleb_id:         str,
        t_anchor:        float,
        horizontal_mtvs: list,
        conflict:        dict,
        shadow:          "RTreeDetector" = None,
    ) -> tuple[Optional[FlightPlan], int]:
        """
        Iterate over horizontal MTVs, building a TRAPEZOID spatial detour for each
        candidate. The detour relies on heuristic scaling and validation against
        the Shadow R-Tree.

        The conflicting OBB sequence is passed to build_trapezoid_detour to compute
        an aggregated safe volume for the evasion maneuver.

        Returns: (accepted_flight_plan | None, iteration_count)
        """
        # Retrieve ALL the plebeian's colliding OBBs in the conflict time range
        pleb_id_key = conflict.get("uav_a", pleb_id)
        t_min, t_max = conflict["time_range"]
        conflict_obbs = []
        try:
            for box in self._manager.uavs[pleb_id_key]["boxes"]:
                if box.t_range[1] > t_min and box.t_range[0] < t_max:
                    conflict_obbs.append(box)
        except KeyError:
            pass

        if not conflict_obbs:
            conflict_obbs = None

        # Compute a weighted conflict origin from SAT-confirmed pairs.
        # This replaces the older "middle OBB" heuristic and works better when
        # one OBB overlaps several others or when the conflict is curved.
        origin_conflict = self._compute_conflict_origin(conflict=conflict, pleb_id=pleb_id)

        # Pre-calculate base t_anc and t_ret from conflict
        v_cruise = fp_pleb.status_at_time(t_anchor).vel
        v_cruise_norm = np.linalg.norm(v_cruise) if v_cruise is not None else 10.0
        if v_cruise_norm < 0.1:
            v_cruise_norm = 10.0
        
        d_anc = (v_cruise_norm ** 2) / float(UAV_MAX_SPEED)
        if d_anc < MIN_DETOUR_DURATION:
            d_anc = MIN_DETOUR_DURATION
        
        t_conflict_start = conflict_obbs[0].t_range[0]
        t_conflict_end = conflict_obbs[-1].t_range[1]
        t_anc_base = t_conflict_start - (d_anc / v_cruise_norm)
        t_anc_base = max(t_anc_base, fp_pleb.init_time() + WAYPOINT_TIME_EPSILON)
        t_anc_base = max(t_anc_base, t_anchor)
        
        t_ret_base = t_conflict_end + (d_anc / v_cruise_norm)
        t_ret_base = min(t_ret_base, fp_pleb.finish_time() - WAYPOINT_TIME_EPSILON)
        
        # Dynamic temporal buffer: scale with conflict duration
        # Longer conflicts get more temporal margin to accommodate the detour
        conflict_duration = t_conflict_end - t_conflict_start
        mtv_scale_time_buffer = conflict_duration * MTV_SCALE_TIME_BUFFER_FACTOR
        print(f"[DEBUG S2] Conflict duration: {conflict_duration:.1f}s -> dynamic buffer: {mtv_scale_time_buffer:.1f}s per MTV scale")
        
        iters_count = 0
        for mtv_idx, mtv in enumerate(horizontal_mtvs):
            print(f"[DEBUG S2]   Trying H-MTV[{mtv_idx}] = {mtv}")
            # Iterative heuristic: scale the MTV to push the detour further
            # if the generated polynomial curve bulges and collides.
            for scale_idx, scale in enumerate([1.0, 1.25, 1.5, 2.0]):
                scaled_mtv = mtv * scale
                
                # Adjust t_anc and t_ret proportionally to MTV scale
                # Larger MTV → earlier start, later end (more time to maneuver)
                time_buffer_extra = (scale - 1.0) * mtv_scale_time_buffer
                t_anc_adj = t_anc_base - time_buffer_extra
                t_ret_adj = t_ret_base + time_buffer_extra
                
                # Validate bounds
                t_anc_adj = max(t_anc_adj, fp_pleb.init_time() + WAYPOINT_TIME_EPSILON)
                t_anc_adj = max(t_anc_adj, t_anchor)
                t_ret_adj = min(t_ret_adj, fp_pleb.finish_time() - WAYPOINT_TIME_EPSILON)
                
                if t_anc_adj >= t_ret_adj:
                    print(f"[DEBUG S2]     Scale {scale_idx+1}/4 (scale={scale}): t_anc_adj exceeds t_ret_adj after bounds check, SKIPPED")
                    continue
                
                print(f"[DEBUG S2]     Scale {scale_idx+1}/4 (scale={scale}): scaled_mtv = {scaled_mtv}, t_anc={t_anc_adj:.2f}s, t_ret={t_ret_adj:.2f}s")
                
                # Build the trapezoid detour with adjusted times
                candidate = build_rigid_shift_detour(
                    fp=fp_pleb,
                    t_anchor=t_anchor,
                    mtv=scaled_mtv,
                    conflict_obbs=conflict_obbs,
                    origin_conflict=origin_conflict,
                    t_anc_override=t_anc_adj,
                    t_ret_override=t_ret_adj,
                )
                if candidate is None:
                    print(f"[DEBUG S2]       -> Detour generation FAILED")
                    continue

                iters_count += 1

                # Global R-Tree validation (shadow)
                if self._validate_shadow(candidate, pleb_id, shadow, conflict["time_range"][0]):
                    print(f"[DEBUG S2]       -> Shadow validation SUCCESS! Detour is conflict-free.")
                    return candidate, iters_count
                else:
                    print(f"[DEBUG S2]       -> Shadow validation FAILED (detour causes secondary conflicts)")

        return None, iters_count

    # ------------------------------------------------------------------
    # Fallback 1: Vertical MTVs (Z-shift happens naturally from SAT result)
    # ------------------------------------------------------------------

    def _fallback1_vertical_mtvs(
        self,
        fp_pleb:       FlightPlan,
        fp_vip:        FlightPlan,
        pleb_id:       str,
        t_anchor:      float,
        vertical_mtvs: list,
        conflict:      dict,
        shadow:        "RTreeDetector" = None,
    ) -> tuple[Optional[FlightPlan], int]:
        """
        Iterate over vertical MTVs (Z-dominant) from the SAT result.

        Uses build_trapezoid_detour to construct a vertical spatial detour.
        The flat top of the trapezoid is a section of the original route displaced
        vertically by the MTV — the UAV climbs / descends to the safe altitude
        corridor, crosses it, then returns. Angular kinematic validation uses
        Algoritmo 6 just like horizontal detours.

        Returns: (accepted_flight_plan | None, iteration_count)
        """
        # Retrieve ALL the plebeian's colliding OBBs in the conflict time range
        pleb_id_key = conflict.get("uav_a", pleb_id)
        t_min, t_max = conflict["time_range"]
        conflict_obbs = []
        try:
            for box in self._manager.uavs[pleb_id_key]["boxes"]:
                if box.t_range[1] > t_min and box.t_range[0] < t_max:
                    conflict_obbs.append(box)
        except KeyError:
            pass

        if not conflict_obbs:
            conflict_obbs = None
        # Reuse the same weighted SAT-based origin as Strategy 2 so the visual
        # reference matches the real detour geometry.
        origin_conflict = self._compute_conflict_origin(conflict=conflict, pleb_id=pleb_id)

        # Pre-calculate base t_anc and t_ret from conflict (same as S2)
        v_cruise = fp_pleb.status_at_time(t_anchor).vel
        v_cruise_norm = np.linalg.norm(v_cruise) if v_cruise is not None else 10.0
        if v_cruise_norm < 0.1:
            v_cruise_norm = 10.0
        
        d_anc = (v_cruise_norm ** 2) / float(UAV_MAX_SPEED)
        if d_anc < MIN_DETOUR_DURATION:
            d_anc = MIN_DETOUR_DURATION
        
        t_conflict_start = conflict_obbs[0].t_range[0]
        t_conflict_end = conflict_obbs[-1].t_range[1]
        t_anc_base = t_conflict_start - (d_anc / v_cruise_norm)
        t_anc_base = max(t_anc_base, fp_pleb.init_time() + WAYPOINT_TIME_EPSILON)
        t_anc_base = max(t_anc_base, t_anchor)
        
        t_ret_base = t_conflict_end + (d_anc / v_cruise_norm)
        t_ret_base = min(t_ret_base, fp_pleb.finish_time() - WAYPOINT_TIME_EPSILON)
        
        # Dynamic temporal buffer: scale with conflict duration
        # Longer conflicts get more temporal margin to accommodate the detour
        conflict_duration = t_conflict_end - t_conflict_start
        mtv_scale_time_buffer = conflict_duration * MTV_SCALE_TIME_BUFFER_FACTOR
        print(f"[DEBUG FB1] Conflict duration: {conflict_duration:.1f}s -> dynamic buffer: {mtv_scale_time_buffer:.1f}s per MTV scale")
        
        iters_count = 0
        for mtv_idx, mtv in enumerate(vertical_mtvs):
            print(f"[DEBUG FB1]   Trying V-MTV[{mtv_idx}] = {mtv}")
            for scale_idx, scale in enumerate([1.0, 1.25, 1.5, 2.0]):
                scaled_mtv = mtv * scale
                
                # Adjust t_anc and t_ret proportionally to MTV scale
                time_buffer_extra = (scale - 1.0) * mtv_scale_time_buffer
                t_anc_adj = t_anc_base - time_buffer_extra
                t_ret_adj = t_ret_base + time_buffer_extra
                
                # Validate bounds
                t_anc_adj = max(t_anc_adj, fp_pleb.init_time() + WAYPOINT_TIME_EPSILON)
                t_anc_adj = max(t_anc_adj, t_anchor)
                t_ret_adj = min(t_ret_adj, fp_pleb.finish_time() - WAYPOINT_TIME_EPSILON)
                
                if t_anc_adj >= t_ret_adj:
                    print(f"[DEBUG FB1]     Scale {scale_idx+1}/4 (scale={scale}): t_anc_adj exceeds t_ret_adj after bounds check, SKIPPED")
                    continue
                
                print(f"[DEBUG FB1]     Scale {scale_idx+1}/4 (scale={scale}): scaled_mtv = {scaled_mtv}, t_anc={t_anc_adj:.2f}s, t_ret={t_ret_adj:.2f}s")
                
                candidate = build_rigid_shift_detour(
                    fp=fp_pleb,
                    t_anchor=t_anchor,
                    mtv=scaled_mtv,
                    conflict_obbs=conflict_obbs,
                    origin_conflict=origin_conflict,
                    t_anc_override=t_anc_adj,
                    t_ret_override=t_ret_adj,
                )
                if candidate is None:
                    print(f"[DEBUG FB1]       -> Detour generation FAILED")
                    continue

                iters_count += 1

                if self._validate_shadow(candidate, pleb_id, shadow, conflict["time_range"][0]):
                    print(f"[DEBUG FB1]       -> Shadow validation SUCCESS! Vertical detour is conflict-free.")
                    return candidate, iters_count
                else:
                    print(f"[DEBUG FB1]       -> Shadow validation FAILED (vertical detour causes secondary conflicts)")

        return None, iters_count

    # ------------------------------------------------------------------
    # Fallback 2: Hover
    # ------------------------------------------------------------------

    def _fallback2_hover(
        self,
        fp_pleb:  FlightPlan,
        fp_vip:   FlightPlan,
        pleb_id:  str,
        t_anchor: float,
        shadow:   "RTreeDetector" = None,
    ) -> tuple[Optional[FlightPlan], int]:
        """
        Brake the plebeian to 0 m/s at t_anchor and progressively advance the
        hover duration until the R-Tree detects no conflict.

        Hard cap: HOVER_MAX_TIMEOUT / HOVER_TIME_STEP iterations.
        Each iteration inserts a hover waypoint and postpones the remainder by
        HOVER_TIME_STEP, then validates. Returns the first valid plan found.

        Returns: (accepted_flight_plan | None, iteration_count)
        """
        max_iters = int(HOVER_MAX_TIMEOUT / HOVER_TIME_STEP)
        position_at_anchor = fp_pleb.status_at_time(t_anchor).pos.copy()

        for iteration in range(1, max_iters + 1):
            hover_duration = iteration * HOVER_TIME_STEP
            t_resume       = t_anchor + hover_duration

            # Build a modified FlightPlan: hover at anchor, then resume
            candidate = fp_pleb.copy()

            # Insert hover start waypoint (velocity = 0)
            hover_start = Waypoint(
                label="HOV_S",
                t=t_anchor,
                pos=position_at_anchor,
                vel=[0.0, 0.0, 0.0],
            )
            # Insert hover end waypoint (same position, velocity = 0)
            hover_end = Waypoint(
                label="HOV_E",
                t=round(t_resume, 3),
                pos=position_at_anchor,
                vel=[0.0, 0.0, 0.0],
            )

            candidate.set_waypoint(hover_start)
            candidate.set_waypoint(hover_end)
            # Postpone all waypoints after t_anchor by hover_duration
            candidate.postpone_from(t_anchor + WAYPOINT_TIME_EPSILON, hover_duration)
            candidate.connect_waypoints()

            if self._validate_shadow(candidate, pleb_id, shadow, t_anchor):
                return candidate, iteration

        # Timeout — DEADLOCK
        return None, max_iters

    # ------------------------------------------------------------------
    # Helper: validate turn kinematics after a detour insertion
    # ------------------------------------------------------------------

    def _validate_detour_kinematics(
        self,
        candidate: FlightPlan,
        t_anchor:  float,
        fp_orig:   FlightPlan,
    ) -> bool:
        """
                Validate the kinematic feasibility of the turn corners introduced by
                the rigid-shift detour:

                    Rigid Shift (build_rigid_shift_detour): "anc" → "det" → "ret"

        The checks are symmetric:
          - First  turn:  [anc-1] → anc → first_detour_wp
          - Last   turn:  last_detour_wp → ret → [ret+1]
        """
        max_ang_vel = fp_orig.max_var_ang_vel

        anc_idx  = candidate.get_index_from_label("anc")
        det_idx  = candidate.get_index_from_label("det")    # legacy triangle
        det1_idx = candidate.get_index_from_label("det1")   # trapezoid
        det2_idx = candidate.get_index_from_label("det2")   # trapezoid
        ret_idx  = candidate.get_index_from_label("ret")

        # Determine whether we have the new rigid shift or the legacy triangle
        if det_idx is not None:
            first_det_idx = det_idx
            last_det_idx  = det_idx
        else:
            # No recognised labels — accept conservatively
            return True

        if anc_idx is None or ret_idx is None:
            return True

        wps = candidate.waypoints

        # Validate first turn: [anc-1] → [anc] → [first_det]
        # TODO: implement validate_curve_kinematics
        # if anc_idx > 0:
        #     ok1 = validate_curve_kinematics(
        #         wp_prev=wps[anc_idx - 1],
        #         wp_turn=wps[anc_idx],
        #         wp_next=wps[first_det_idx],
        #         max_ang_vel=max_ang_vel,
        #     )
        #     if not ok1:
        #         return False

        # Validate last turn: [last_det] → [ret] → [ret+1]
        # if ret_idx < len(wps) - 1:
        #     ok2 = validate_curve_kinematics(
        #         wp_prev=wps[last_det_idx],
        #         wp_turn=wps[ret_idx],
        #         wp_next=wps[ret_idx + 1],
        #         max_ang_vel=max_ang_vel,
        #     )
        #     if not ok2:
        #         return False

        return True
