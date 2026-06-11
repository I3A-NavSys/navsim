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
     1. VIP IMMUTABILITY: The VIP flight plan is never modified by
         the resolver. All resolution actions apply only to the plebeian's plan. 
         However, it can be aborted if no valid plebeian maneuver exists (DEADLOCK).

     2. WCET ANCHOR: All maneuvers start at t_anchor >= t_start + ANCHOR_DELTA.
         The resolver never operates before the UAV's scheduled departure time.

     3. BOUNDED LOOPS: Every loop in this module has a hard iteration cap.
         No while-true loops exist. The Fallback 2 hovering loop is explicitly
         capped at HOVER_MAX_TIMEOUT / HOVER_TIME_STEP iterations.

     4. CANDIDATE-QUERY VALIDATION: All candidate plans are validated against
         the live detector by generating candidate swept boxes in memory and
         querying the active detector without modifying the live index.
"""

from __future__ import annotations

import time
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, TYPE_CHECKING

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
    UAV_MAX_ACCEL,
    CRUISE_SPEED_FALLBACK,
    MIN_DETOUR_DURATION,
    MTV_SCALE_TIME_BUFFER_FACTOR,
    S1_TIME_SHIFTS,
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
        deadlock_type: Cause of a DEADLOCK outcome, when applicable.
                       One of: "anchor_before_conflict", "anchor_after_finish",
                       "strategy_exhausted", "hover_timeout".
        new_fp_pleb:   The new FlightPlan for the plebeian UAV (None on DEADLOCK).
        new_fp_vip:    Always None. The VIP is never modified (only the plebeian).
        iterations:    Total number of candidate plans evaluated across all phases.
        phase_iterations:
                       Per-phase candidate counts used by the resolver.
        message:       Human-readable summary for logging / debugging.
        phase_times:   Breakdown of the resolve time by phase in milliseconds.
        pass_history:  Per-pass timing snapshots when a caller resolves until clear.
    """
    success:       bool                  = False
    strategy_used: str                   = "NONE"
    deadlock_type: str                   = "none"
    new_fp_pleb:   Optional[FlightPlan]  = None
    new_fp_vip:    Optional[FlightPlan]  = None
    iterations:    int                   = 0
    phase_iterations: Dict[str, int]     = field(default_factory=dict)
    message:       str                   = ""
    phase_times:   Dict[str, float]      = field(default_factory=dict)
    pass_history:  List[Dict[str, object]] = field(default_factory=list)


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
        resolve_t0 = time.perf_counter()
        phase_times: Dict[str, float] = {}
        phase_iterations: Dict[str, int] = {}

        def _finalize(result: ResolveResult) -> ResolveResult:
                # Merge last validation metrics if available (populated by candidate-query validation)
            if hasattr(self, "_last_validation_info") and isinstance(self._last_validation_info, dict):
                phase_times.update(self._last_validation_info)
                # Clear it to avoid leaking between resolves
                delattr(self, "_last_validation_info")

            phase_times["resolve_total_ms"] = (time.perf_counter() - resolve_t0) * 1000.0
            result.phase_times = dict(phase_times)
            result.phase_iterations = dict(phase_iterations)
            return result

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
                    deadlock_type="anchor_before_conflict",
                    message=f"t_anchor cannot be placed before t_conflict={t_conflict:.2f}s "
                            f"(t_start={t_start:.2f}s, ANCHOR_DELTA={ANCHOR_DELTA}s). "
                            f"Mission abort.",
                )

        # Guard: anchor must be before plebeian finishes its flight
        if t_anchor >= fp_pleb.finish_time():
            return ResolveResult(
                success=False,
                strategy_used="DEADLOCK",
                deadlock_type="anchor_after_finish",
                message=f"t_anchor={t_anchor:.2f}s is beyond plebeian flight end "
                        f"({fp_pleb.finish_time():.2f}s). Mission abort.",
            )

        # =================================================================
        # Step 1: Validation uses a candidate-query against the live R-Tree.
        # Candidate flight plans are generated in memory and validated by
        # querying the active detector; no entries are inserted into or
        # removed from the live index during validation.
        # =================================================================
        phase_times["validation_ms"] = 0.0

        def _check_valid(candidate: FlightPlan) -> bool:
            return self._validate_candidate_query(candidate, pleb_id, t_conflict)

        # =================================================================
        # Step 2: Strategy 1 — Kinematic Bounding (velocity-only)
        # =================================================================
        phase_t0 = time.perf_counter()
        result_s1, s1_iterations = run_strategy1(
            fp_pleb=fp_pleb,
            t_conflict=t_conflict,
            validator_fn=_check_valid,
        )
        phase_times["s1_ms"] = (time.perf_counter() - phase_t0) * 1000.0
        phase_iterations["s1"] = s1_iterations
        total_iterations += s1_iterations

        if result_s1 is not None:
            return _finalize(ResolveResult(
                success=True,
                strategy_used="S1",
                new_fp_pleb=result_s1,
                new_fp_vip=None,
                iterations=total_iterations,
                phase_iterations=dict(phase_iterations),
                message="Strategy 1 (Kinematic Bounding) succeeded.",
            ))
        else:
            print(f"[DEBUG S1] Kinematic Bounding FAILED for {pleb_id}")
            print(f"[DEBUG S1] Could not resolve conflict by velocity adjustments alone")

        # =================================================================
        # Step 3: Generate SAT MTV candidates for spatial strategies
        # =================================================================
        phase_t0 = time.perf_counter()
        sat_result: SATResult = generate_mtv_candidates(
            conflict=conflict,
            manager=self._manager,
            flight_plan_pleb=fp_pleb,
        )
        phase_times["sat_ms"] = (time.perf_counter() - phase_t0) * 1000.0

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
        phase_t0 = time.perf_counter()
        s2_result, iters = self._strategy2_horizontal_mtvs(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
            horizontal_mtvs=sat_result.horizontal_mtvs,
            conflict=conflict,
        )
        phase_times["s2_ms"] = (time.perf_counter() - phase_t0) * 1000.0
        phase_iterations["s2"] = iters
        total_iterations += iters

        if s2_result is not None:
            print(f"[DEBUG S2] SUCCESS! Resolved {pleb_id} with S2 after {iters} iterations")
            return _finalize(ResolveResult(
                success=True,
                strategy_used="S2",
                new_fp_pleb=s2_result,
                new_fp_vip=None,
                iterations=total_iterations,
                phase_iterations=dict(phase_iterations),
                message="Strategy 2 (Horizontal Path Stretch) succeeded.",
            ))
        else:
            print(f"[DEBUG S2] FAILED! No valid horizontal detour found after {iters} iterations")

        # =================================================================
        # Step 5: Fallback 1 — Vertical MTVs
        # build_trapezoid_detour handles Z-dominant MTVs the same way as
        # horizontal ones: the flat top is displaced vertically.
        # =================================================================
        print(f"[DEBUG FB1] Attempting Fallback 1 (Vertical MTVs) for {pleb_id}...")
        phase_t0 = time.perf_counter()
        fb1_result, iters = self._fallback1_vertical_mtvs(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
            vertical_mtvs=sat_result.vertical_mtvs,
            conflict=conflict,
        )
        phase_times["fb1_ms"] = (time.perf_counter() - phase_t0) * 1000.0
        phase_iterations["fb1"] = iters
        total_iterations += iters

        if fb1_result is not None:
            print(f"[DEBUG FB1] SUCCESS! Resolved {pleb_id} with FB1 after {iters} iterations")
            return _finalize(ResolveResult(
                success=True,
                strategy_used="FB1",
                new_fp_pleb=fb1_result,
                new_fp_vip=None,
                iterations=total_iterations,
                phase_iterations=dict(phase_iterations),
                message="Fallback 1 (Vertical MTVs) succeeded.",
            ))

        # =================================================================
        # Step 6: Fallback 2 — Hover
        # =================================================================
        phase_t0 = time.perf_counter()
        fb2_result, iters = self._fallback2_hover(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
        )
        phase_times["fb2_ms"] = (time.perf_counter() - phase_t0) * 1000.0
        phase_iterations["fb2"] = iters
        total_iterations += iters

        if fb2_result is not None:
            return _finalize(ResolveResult(
                success=True,
                strategy_used="FB2",
                new_fp_pleb=fb2_result,
                new_fp_vip=None,
                iterations=total_iterations,
                phase_iterations=dict(phase_iterations),
                message="Fallback 2 (Hover) succeeded.",
            ))

        # =================================================================
        # Step 7: DEADLOCK — all strategies exhausted
        # =================================================================
        return _finalize(ResolveResult(
            success=False,
            strategy_used="DEADLOCK",
            deadlock_type="hover_timeout",
            new_fp_pleb=None,
            new_fp_vip=None,
            iterations=total_iterations,
            phase_iterations=dict(phase_iterations),
            message=(
                f"DEADLOCK: All resolution strategies failed for plebeian '{pleb_id}' "
                f"vs VIP '{vip_id}'. Mission abort required."
            ),
        ))

    # ------------------------------------------------------------------
    # Internal: Validate candidate by querying the live R-Tree (no modifications to the index)
    # ------------------------------------------------------------------

    def _validate_candidate_query(
        self,
        candidate_fp: FlightPlan,
        pleb_id: str,
        t_conflict_being_solved: float = 0.0,
    ) -> bool:
        """
        Validate a candidate plebeian plan by query-only validation against the live detector.

        The candidate's swept boxes are generated in memory and checked against
        the currently registered UAVs using broad-phase R-Tree queries followed
        by narrow-phase SAT checks. No insertion or deletion is performed on
        the live R-Tree during validation; collisions against the plebeian's
        own currently registered route are explicitly ignored.

        Forward progress guarantee:
        If the route still contains conflicts, the system accepts it only when
        the earliest remaining conflict is comfortably after the one currently
        being solved. This lets CentralManager resolve separated conflict
        periods in the same flight plan iteratively without deadlock.
        """
        manager = self._manager
        original_data = manager.uavs.get(pleb_id)
        original_boxes_count = len(original_data["boxes"]) if original_data is not None else 0

        t0 = time.perf_counter()
        candidate_boxes = candidate_fp.generate_swept_boxes_obb(interval=OBB_INTERVAL)
        t_gen = (time.perf_counter() - t0) * 1000.0

        t0 = time.perf_counter()
        conflicts = manager.detect_candidate_conflicts(
            candidate_boxes=candidate_boxes,
            candidate_uav_id=pleb_id,
            early_exit=False,
        )
        t_detect = (time.perf_counter() - t0) * 1000.0

        print(
            f"[VALIDATION] pleb={pleb_id} candidate-query: "
            f"gen={len(candidate_boxes)}boxes {t_gen:.1f}ms, "
            f"detect_time={t_detect:.1f}ms conflicts={len(conflicts)}"
        )

        # Store validation metrics for benchmark aggregation.
        self._last_validation_info = {
            "validation_method": "candidate-query",
            "candidate_boxes": int(len(candidate_boxes)),
            "original_boxes": int(original_boxes_count),
            "candidate_gen_ms": float(t_gen),
            "detect_ms": float(t_detect),
        }

        if len(conflicts) == 0:
            return True

        # Check for forward progress.
        conflicts.sort(key=lambda c: c["time_range"][0])
        earliest_new_conflict = conflicts[0]["time_range"][0]
        return earliest_new_conflict >= t_conflict_being_solved + FORWARD_PROGRESS_MARGIN

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
    ) -> tuple[Optional[FlightPlan], int]:
        """
        Iterate over horizontal MTVs, building a TRAPEZOID spatial detour for each
        candidate. The detour relies on heuristic scaling and validation against
        the live R-Tree through candidate queries.

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

        # Pre-calculate base t_anc and t_ret from conflict
        v_cruise = fp_pleb.status_at_time(t_anchor).vel
        v_cruise_norm = np.linalg.norm(v_cruise) if v_cruise is not None else CRUISE_SPEED_FALLBACK
        if v_cruise_norm < 0.1:
            v_cruise_norm = CRUISE_SPEED_FALLBACK

        # Compute a minimum detour duration (seconds) based on available
        # cruise speed and maximum acceleration. This reserves a symmetric
        # time window before and after the conflict for the maneuver.
        detour_time = max(v_cruise_norm / float(UAV_MAX_ACCEL), MIN_DETOUR_DURATION)

        t_conflict_start = conflict_obbs[0].t_range[0]
        t_conflict_end = conflict_obbs[-1].t_range[1]

        # Set the detour start at t_anchor (or init floor) and compute a
        # symmetric separation delta to apply to the conflict end. The
        # separation is at least `detour_time` to guarantee minimum duration
        # for the maneuver.
        t_anc_base = max(t_anchor, fp_pleb.init_time() + WAYPOINT_TIME_EPSILON)
        separation = max(t_conflict_start - t_anc_base, detour_time)
        t_ret_base = t_conflict_end + separation
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
                
                # Expand detour window symmetrically: both start and end move outward.
                # Larger MTV → earlier start, later end (more time to maneuver, smoother curves).
                time_buffer_extra = (scale - 1.0) * mtv_scale_time_buffer
                t_detour_start_adj = t_anc_base - time_buffer_extra
                t_detour_end_adj = t_ret_base + time_buffer_extra
                
                # Validate bounds: start >= anchor (WCET floor), end <= finish
                t_detour_start_adj = max(t_detour_start_adj, fp_pleb.init_time() + WAYPOINT_TIME_EPSILON)
                t_detour_start_adj = max(t_detour_start_adj, t_anchor)
                t_detour_end_adj = min(t_detour_end_adj, fp_pleb.finish_time() - WAYPOINT_TIME_EPSILON)
                
                if t_detour_start_adj >= t_detour_end_adj:
                    print(f"[DEBUG S2]     Scale {scale_idx+1}/4 (scale={scale}): t_detour_start_adj exceeds t_detour_end_adj after bounds check, SKIPPED")
                    continue
                
                print(f"[DEBUG S2]     Scale {scale_idx+1}/4 (scale={scale}): scaled_mtv = {scaled_mtv}, t_start={t_detour_start_adj:.2f}s, t_end={t_detour_end_adj:.2f}s")
                
                # Build the trapezoid detour with adjusted times
                candidate = build_rigid_shift_detour(
                    fp=fp_pleb,
                    t_detour_starthor=t_anchor,
                    mtv=scaled_mtv,
                    conflict_obbs=conflict_obbs,
                    t_detour_start_override=t_detour_start_adj,
                    t_detour_end_override=t_detour_end_adj,
                )
                if candidate is None:
                    print(f"[DEBUG S2]       -> Detour generation FAILED")
                    continue

                iters_count += 1

                # Global R-Tree validation using the live detector.
                if self._validate_candidate_query(candidate, pleb_id, conflict["time_range"][0]):
                    print(f"[DEBUG S2]       -> Live validation SUCCESS! Detour is conflict-free.")
                    return candidate, iters_count
                else:
                    print(f"[DEBUG S2]       -> Live validation FAILED (detour causes secondary conflicts)")

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
        # Pre-calculate base t_detour_start and t_detour_end from conflict (same as S2)
        v_cruise = fp_pleb.status_at_time(t_anchor).vel
        v_cruise_norm = np.linalg.norm(v_cruise) if v_cruise is not None else CRUISE_SPEED_FALLBACK
        if v_cruise_norm < 0.1:
            v_cruise_norm = CRUISE_SPEED_FALLBACK

        # Compute a minimum detour duration (seconds) based on available
        # cruise speed and maximum acceleration. This reserves a symmetric
        # time window before and after the conflict for the maneuver.
        detour_time = max(v_cruise_norm / float(UAV_MAX_ACCEL), MIN_DETOUR_DURATION)

        t_conflict_start = conflict_obbs[0].t_range[0]
        t_conflict_end = conflict_obbs[-1].t_range[1]

        # Set the detour start at t_anchor (or init floor) and compute a
        # symmetric separation delta to apply to the conflict end. The
        # separation is at least `detour_time` to guarantee minimum duration
        # for the maneuver.
        t_anc_base = max(t_anchor, fp_pleb.init_time() + WAYPOINT_TIME_EPSILON)
        separation = max(t_conflict_start - t_anc_base, detour_time)
        t_ret_base = t_conflict_end + separation
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
                
                # Expand detour window symmetrically: both start and end move outward.
                # Larger MTV → earlier start, later end (more time to maneuver, smoother curves).
                time_buffer_extra = (scale - 1.0) * mtv_scale_time_buffer
                t_detour_start_adj = t_anc_base - time_buffer_extra
                t_detour_end_adj = t_ret_base + time_buffer_extra
                
                # Validate bounds: start >= anchor (WCET floor), end <= finish
                t_detour_start_adj = max(t_detour_start_adj, fp_pleb.init_time() + WAYPOINT_TIME_EPSILON)
                t_detour_start_adj = max(t_detour_start_adj, t_anchor)
                t_detour_end_adj = min(t_detour_end_adj, fp_pleb.finish_time() - WAYPOINT_TIME_EPSILON)
                
                if t_detour_start_adj >= t_detour_end_adj:
                    print(f"[DEBUG FB1]     Scale {scale_idx+1}/4 (scale={scale}): t_detour_start_adj exceeds t_detour_end_adj after bounds check, SKIPPED")
                    continue
                
                print(f"[DEBUG FB1]     Scale {scale_idx+1}/4 (scale={scale}): scaled_mtv = {scaled_mtv}, t_start={t_detour_start_adj:.2f}s, t_end={t_detour_end_adj:.2f}s")
                
                candidate = build_rigid_shift_detour(
                    fp=fp_pleb,
                    t_detour_starthor=t_anchor,
                    mtv=scaled_mtv,
                    conflict_obbs=conflict_obbs,
                    t_detour_start_override=t_detour_start_adj,
                    t_detour_end_override=t_detour_end_adj,
                )
                if candidate is None:
                    print(f"[DEBUG FB1]       -> Detour generation FAILED")
                    continue

                iters_count += 1

                if self._validate_candidate_query(candidate, pleb_id, conflict["time_range"][0]):
                    print(f"[DEBUG FB1]       -> Live validation SUCCESS! Vertical detour is conflict-free.")
                    return candidate, iters_count
                else:
                    print(f"[DEBUG FB1]       -> Live validation FAILED (vertical detour causes secondary conflicts)")

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
    ) -> tuple[Optional[FlightPlan], int]:
        """
        Fallback 2 (Hover) implementation.

                FB2 inserts an explicit hover segment anchored at `t_anchor` and then
                postpones the unresolved suffix of the flight plan so the already flown
                prefix is preserved.

        Rationale / notes:
                - The already-flown prefix is left untouched.
                - The hover segment is represented by two fixed markers, `HOV_S` and
                    `HOV_E`, with zero velocity at the hover position.
                - The remaining suffix is shifted forward in time, which is a closer
                    match to a real replanning command sent to a UAV already in motion.
        - We keep the same iteration caps (`HOVER_MAX_TIMEOUT`,
          `HOVER_TIME_STEP`) and the same kinematic validation via
          `connect_waypoints()`.

        Returns: (accepted_flight_plan | None, iteration_count)
        """
        max_iters = int(HOVER_MAX_TIMEOUT / HOVER_TIME_STEP)
        hover_position = np.array(fp_pleb.status_at_time(t_anchor).pos, dtype=float)

        for iteration in range(1, max_iters + 1):
            hover_duration = iteration * HOVER_TIME_STEP

            # Build an explicit hover segment at the anchor and postpone only
            # the remaining suffix so already-flown waypoints stay fixed.
            candidate = fp_pleb.copy()
            candidate.set_waypoint(
                Waypoint(
                    label="HOV_S",
                    t=t_anchor,
                    pos=hover_position,
                    vel=np.zeros(3),
                )
            )
            candidate.set_waypoint(
                Waypoint(
                    label="HOV_E",
                    t=t_anchor + hover_duration,
                    pos=hover_position,
                    vel=np.zeros(3),
                )
            )
            candidate.postpone_from(t_anchor, hover_duration)

            try:
                candidate.connect_waypoints(strict=True)
            except ValueError as exc:
                # Physically infeasible to rejoin the original timing with this
                # postponement (insufficient accel/decel margins). Try a longer
                # postponement instead of aborting the cascade.
                print(f"[DEBUG FB2-SIMPLE] Iter {iteration}: infeasible postponed candidate -> {exc}")
                continue

            # Validate candidate against live detector (candidate-query)
            if self._validate_candidate_query(candidate, pleb_id, t_anchor):
                return candidate, iteration

        # Timeout — DEADLOCK
        return None, max_iters

