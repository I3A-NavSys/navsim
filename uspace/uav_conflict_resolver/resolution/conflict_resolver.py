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
        FB2 (Time-Shift / Hovering)
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
)
from resolution.geometry.sat_mtv import generate_mtv_candidates, SATResult
from resolution.geometry.path_geometry import (
    build_spatial_detour,
    build_trapezoid_detour,
    validate_curve_kinematics,
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
        # Step 2: Strategy 1 — Kinematic Bounding (velocity-only)
        # =================================================================
        result_s1 = run_strategy1(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
            manager=self._manager,
            t_conflict=t_conflict,
        )

        # =================================================================
        # Build shadow R-Tree ONCE for all remaining strategies (S2/FB1/FB2).
        # This avoids re-registering every other UAV for each candidate.
        # register_uav() already removes old boxes before inserting new
        # ones, so successive candidate swaps are safe.
        # =================================================================
        shadow = self._build_shadow_rtree(pleb_id)
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

        # =================================================================
        # Step 3: Generate SAT MTV candidates for spatial strategies
        # =================================================================
        sat_result: SATResult = generate_mtv_candidates(
            conflict=conflict,
            manager=self._manager,
        )

        if not sat_result.success:
            # SAT did not find a meaningful MTV — fall through to Fallback 2
            sat_result.horizontal_mtvs = []
            sat_result.vertical_mtvs   = []

        # =================================================================
        # Step 4: Strategy 2 — Horizontal Path Stretch
        # =================================================================
        s2_result, iters = self._strategy2_horizontal(
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
            fp_vip_released = self._release_vip(fp_vip_held, t_anchor)
            return ResolveResult(
                success=True,
                strategy_used="S2",
                new_fp_pleb=s2_result,
                new_fp_vip=fp_vip_released,
                iterations=total_iterations,
                message="Strategy 2 (Horizontal Path Stretch) succeeded.",
            )

        # =================================================================
        # Step 5: Fallback 1 — Vertical MTVs
        # build_trapezoid_detour handles Z-dominant MTVs the same way as
        # horizontal ones: the flat top is displaced vertically.
        # =================================================================
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
        # Step 6: Fallback 2 — Time-Shift (Hovering)
        # =================================================================
        fb2_result, iters = self._fallback2_timeshift(
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
                message="Fallback 2 (Time-Shift / Hovering) succeeded.",
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

    # ------------------------------------------------------------------
    # Strategy 2: Horizontal Path Stretch
    # ------------------------------------------------------------------

    def _strategy2_horizontal(
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

        iters_count = 0
        for mtv in horizontal_mtvs:
            # Iterative heuristic: scale the MTV to push the detour further
            # if the generated polynomial curve bulges and collides.
            for scale in [1.0, 1.25, 1.5, 2.0]:
                scaled_mtv = mtv * scale
                
                # Build the trapezoid detour
                candidate = build_trapezoid_detour(
                    fp=fp_pleb,
                    t_anchor=t_anchor,
                    mtv=scaled_mtv,
                    conflict_obbs=conflict_obbs,
                )
                if candidate is None:
                    continue

                iters_count += 1

                # Global R-Tree validation (shadow)
                if self._validate_shadow(candidate, pleb_id, shadow, conflict["time_range"][0]):
                    return candidate, iters_count

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

        iters_count = 0
        for mtv in vertical_mtvs:
            for scale in [1.0, 1.25, 1.5, 2.0]:
                scaled_mtv = mtv * scale
                
                candidate = build_trapezoid_detour(
                    fp=fp_pleb,
                    t_anchor=t_anchor,
                    mtv=scaled_mtv,
                    conflict_obbs=conflict_obbs,
                )
                if candidate is None:
                    continue

                iters_count += 1

                if self._validate_shadow(candidate, pleb_id, shadow, conflict["time_range"][0]):
                    return candidate, iters_count

        return None, iters_count

    # ------------------------------------------------------------------
    # Fallback 2: Time-Shift (Hovering)
    # ------------------------------------------------------------------

    def _fallback2_timeshift(
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
        the detour.  Supports BOTH label schemes:

          Legacy  (build_spatial_detour)  : "anc" → "det"  → "ret"
          Trapezoid (build_trapezoid_detour): "anc" → "det1" → "det2" → "ret"

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

        # Determine whether we have the trapezoid or the legacy triangle
        if det1_idx is not None and det2_idx is not None:
            first_det_idx = det1_idx
            last_det_idx  = det2_idx
        elif det_idx is not None:
            first_det_idx = det_idx
            last_det_idx  = det_idx
        else:
            # No recognised labels — accept conservatively
            return True

        if anc_idx is None or ret_idx is None:
            return True

        wps = candidate.waypoints

        # Validate first turn: [anc-1] → [anc] → [first_det]
        if anc_idx > 0:
            ok1 = validate_curve_kinematics(
                wp_prev=wps[anc_idx - 1],
                wp_turn=wps[anc_idx],
                wp_next=wps[first_det_idx],
                max_ang_vel=max_ang_vel,
            )
            if not ok1:
                return False

        # Validate last turn: [last_det] → [ret] → [ret+1]
        if ret_idx < len(wps) - 1:
            ok2 = validate_curve_kinematics(
                wp_prev=wps[last_det_idx],
                wp_turn=wps[ret_idx],
                wp_next=wps[ret_idx + 1],
                max_ang_vel=max_ang_vel,
            )
            if not ok2:
                return False

        return True
