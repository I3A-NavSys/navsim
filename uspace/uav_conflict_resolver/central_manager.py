"""
central_manager.py — Public Façade for the UAV Deconfliction System
====================================================================

PURPOSE:
    CentralManager is the single public entry point for the entire deconfliction
    pipeline. External systems (the simulator, the U-Space operator, MQTT handlers)
    interact only with this class.

    It composes:
        RTreeDetector     → 4D R-Tree detection (broad + narrow phase)
        ConflictResolver  → Cascading resolution (S1 → S2 → FB1 → FB2 → DEADLOCK)

TYPICAL USAGE:
    cm = CentralManager()

    # Register all UAVs at mission start
    cm.register_uav("VIP_01",  fp_vip)
    cm.register_uav("PLEB_01", fp_pleb)

    # At each planning cycle, check a specific UAV for conflicts and resolve
    result = cm.check_and_resolve("PLEB_01", priority_map={"VIP_01": 10, "PLEB_01": 1})
    if result and result.success:
        # The manager already committed the new plans internally
        new_pleb_fp = cm.get_flight_plan("PLEB_01")

PRIORITY CONVENTION:
    Only 3 priority levels are used internally: 0 = LOW, 1 = MEDIUM, 2 = HIGH.
    Higher numeric priority wins (VIP > Plebeian). In a conflict between two UAVs,
    the one with LOWER priority is the Plebeian whose plan gets modified.
    If both UAVs share the same priority, the tie is broken deterministically by
    UAV ID: the lexicographically smaller ID becomes the VIP and the larger ID
    becomes the Plebeian.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional

from core.models.flight_plan import FlightPlan
from core.config import OBB_INTERVAL
from detection.rtree_detector import RTreeDetector
from resolution.conflict_resolver import ConflictResolver, ResolveResult

logger = logging.getLogger(__name__)


class CentralManager:
    """
    Unified façade for UAV 4D conflict detection and resolution.

    Attributes:
        _rtree_detector: Internal RTreeDetector (R-Tree + SAT engine).
        _resolver:          ConflictResolver bound to the strategic manager.
        _flight_plans:      Live registry of UAV → FlightPlan mappings.
        _priorities:        UAV → priority (int) mapping. Higher = more important.
    """

    def __init__(self) -> None:
        self._rtree_detector: RTreeDetector = RTreeDetector()
        self._resolver: ConflictResolver = ConflictResolver(self._rtree_detector)
        self._flight_plans: Dict[str, FlightPlan] = {}
        self._priorities:   Dict[str, int]         = {}

    @staticmethod
    def _normalize_priority(priority: int) -> int:
        """
        Clamp any external priority input to the three supported levels.

        External callers may still pass legacy values (for example 5 or 10),
        but internally the system only keeps 0, 1, or 2.
        """
        return max(0, min(2, int(priority)))

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def register_uav(
        self,
        uav_id:     str,
        flight_plan: FlightPlan,
        priority:   int = 0,
        interval:   float = OBB_INTERVAL,
    ) -> None:
        """
        Register (or update) a UAV with its flight plan and priority.

        This adds the UAV to both the internal registry and the R-Tree index.
        If the UAV was already registered, its previous boxes are evicted and
        replaced with the new plan's boxes.

        Args:
            uav_id:      Unique UAV identifier string.
            flight_plan: The UAV's FlightPlan object.
            priority:    Integer priority. Internally clamped to 0, 1, or 2.
                         Default 0 (LOW).
            interval:    OBB sampling interval [s] for R-Tree box generation.
        """
        self._flight_plans[uav_id] = flight_plan
        normalized_priority = self._normalize_priority(priority)
        self._priorities[uav_id]   = normalized_priority
        self._rtree_detector.register_uav(uav_id, flight_plan, interval=interval)
        logger.debug(
            f"[CentralManager] Registered UAV '{uav_id}' "
            f"(priority={priority} -> {normalized_priority})"
        )

    def bulk_register_uavs(
        self,
        uav_entries,
        priority_map: Optional[Dict[str, int]] = None,
        interval: float = OBB_INTERVAL,
    ) -> None:
        """
        Register a complete fleet in one shot and rebuild the detector index.

        Args:
            uav_entries: iterable of (uav_id, flight_plan) pairs.
            priority_map: optional uav_id -> priority mapping.
            interval: OBB sampling interval [s] for all flight plans.
        """
        entries = list(uav_entries)
        self._flight_plans = {uav_id: flight_plan for uav_id, flight_plan in entries}
        self._priorities = {
            uav_id: self._normalize_priority((priority_map or {}).get(uav_id, 0))
            for uav_id, _ in entries
        }
        self._rtree_detector.bulk_register_uavs(entries, interval=interval)
        logger.debug(f"[CentralManager] Bulk registered {len(entries)} UAVs.")

    def get_flight_plan(self, uav_id: str) -> Optional[FlightPlan]:
        """
        Retrieve the current (possibly updated) FlightPlan for a UAV.

        Returns None if the UAV is not registered.
        """
        return self._flight_plans.get(uav_id)

    def remove_uav(self, uav_id: str) -> None:
        """
        Remove a UAV from the manager and the detector.

        This is used when a newly inserted route cannot be accepted.
        """
        self._flight_plans.pop(uav_id, None)
        self._priorities.pop(uav_id, None)
        self._rtree_detector.remove_uav(uav_id)

    # ------------------------------------------------------------------
    # Conflict Detection + Resolution
    # ------------------------------------------------------------------

    def check_and_resolve(
        self,
        target_uav_id: str,
        interval: float = OBB_INTERVAL,
    ) -> Optional[ResolveResult]:
        """
        Detect all conflicts for target_uav_id and, if any are found,
        resolve the most urgent one (earliest time_range[0]).

        The resolution cascade uses the priority map to identify which UAV
        is the VIP and which is the plebeian.

        After a successful resolution the internal registry and R-Tree are
        updated automatically — the caller does not need to call register_uav()
        again.

        Args:
            target_uav_id: The UAV to check. Must already be registered.

        Returns:
            ResolveResult if at least one conflict was found (may be DEADLOCK),
            None if no conflicts were detected (airspace is clear).
        """
        if target_uav_id not in self._flight_plans:
            logger.warning(
                f"[CentralManager] check_and_resolve called for unknown UAV "
                f"'{target_uav_id}'. Returning None."
            )
            return None

        # ------------------------------------------------------------------
        # 1. Detect all conflicts for this UAV
        # ------------------------------------------------------------------
        conflicts = self._rtree_detector.detect_all_conflicts(target_uav_id)
        if not conflicts:
            logger.debug(f"[CentralManager] No conflicts detected for '{target_uav_id}'.")
            return None

        # ------------------------------------------------------------------
        # 2. Pick the most urgent conflict (earliest temporal overlap)
        # ------------------------------------------------------------------
        conflicts.sort(key=lambda c: c["time_range"][0])
        conflict = conflicts[0]
        uav_a_id = conflict["uav_a"]
        uav_b_id = conflict["uav_b"]

        # ------------------------------------------------------------------
        # 3. Determine VIP vs Plebeian by priority
        # ------------------------------------------------------------------
        priority_a = self._priorities.get(uav_a_id, 0)
        priority_b = self._priorities.get(uav_b_id, 0)

        if priority_a >= priority_b:
            if priority_a == priority_b and uav_a_id > uav_b_id:
                vip_id, pleb_id = uav_b_id, uav_a_id
            else:
                vip_id, pleb_id = uav_a_id, uav_b_id
        else:
            vip_id, pleb_id = uav_b_id, uav_a_id

        fp_vip = self._flight_plans[vip_id]
        fp_pleb = self._flight_plans[pleb_id]

        logger.info(
            f"[CentralManager] Conflict at t={conflict['time_range'][0]:.2f}s: "
            f"VIP='{vip_id}' vs Plebeian='{pleb_id}'"
        )

        # ------------------------------------------------------------------
        # 4. Run the resolution cascade
        # ------------------------------------------------------------------
        result: ResolveResult = self._resolver.resolve(
            conflict=conflict,
            fp_pleb=fp_pleb,
            fp_vip=fp_vip,
            pleb_id=pleb_id,
            vip_id=vip_id,
        )

        # ------------------------------------------------------------------
        # 5. Commit the new plans if resolution succeeded
        # ------------------------------------------------------------------
        if result.success:
            logger.info(
                f"[CentralManager] Resolved via {result.strategy_used} "
                f"in {result.iterations} iteration(s). {result.message}"
            )
            if result.new_fp_pleb is not None:
                self.register_uav(
                    pleb_id,
                    result.new_fp_pleb,
                    priority=self._priorities.get(pleb_id, 0),
                    interval=interval,
                )
            if result.new_fp_vip is not None:
                self.register_uav(
                    vip_id,
                    result.new_fp_vip,
                    priority=self._priorities.get(vip_id, 0),
                    interval=interval,
                )
        else:
            logger.error(
                f"[CentralManager] DEADLOCK — {result.message} "
                f"({result.iterations} iterations attempted)"
            )

        return result

    def resolve_until_clear(
        self,
        target_uav_id: str,
        interval: float = OBB_INTERVAL,
        max_passes: int = 300,
    ) -> Optional[ResolveResult]:
        """
        Keep resolving the target UAV until it has no remaining conflicts.

        This is the entry point to use when a newly inserted flight plan may
        contain several separated conflict periods. The method resolves the
        earliest conflict first, commits the updated plan, and then re-checks
        the same UAV again until the route is fully clean or a safety cap is
        reached.
        """
        if target_uav_id not in self._flight_plans:
            logger.warning(
                f"[CentralManager] resolve_until_clear called for unknown UAV "
                f"'{target_uav_id}'. Returning None."
            )
            return None

        last_result: Optional[ResolveResult] = None
        pass_history: List[Dict[str, object]] = []

        for pass_idx in range(max_passes):
            # Re-evaluate the same UAV after each committed fix so separated
            # conflict windows can be handled one by one until the route is clean.
            conflicts = self._rtree_detector.detect_all_conflicts(target_uav_id)
            if not conflicts:
                if last_result is None:
                    logger.debug(f"[CentralManager] No conflicts detected for '{target_uav_id}'.")
                else:
                    logger.info(
                        f"[CentralManager] '{target_uav_id}' is now conflict-free after "
                        f"{pass_idx} pass(es)."
                    )
                    if last_result is not None:
                        last_result.pass_history = list(pass_history)
                return last_result

            last_result = self.check_and_resolve(target_uav_id, interval=interval)
            if last_result is None:
                return None

            remaining_conflicts = self._rtree_detector.detect_all_conflicts(target_uav_id)
            phase_times = dict(last_result.phase_times)
            pass_history.append(
                {
                    "pass_index": pass_idx + 1,
                    "conflicts_before": len(conflicts),
                    "remaining_conflicts_after": len(remaining_conflicts),
                    "strategy_used": last_result.strategy_used,
                    "success": last_result.success,
                    "iterations": last_result.iterations,
                    "phase_iterations": dict(last_result.phase_iterations),
                    "phase_times": phase_times,
                    "resolve_total_ms": float(phase_times.get("resolve_total_ms", 0.0)),
                    "resolve_phase_sum_ms": float(
                        phase_times.get("s1_ms", 0.0)
                        + phase_times.get("sat_ms", 0.0)
                        + phase_times.get("s2_ms", 0.0)
                        + phase_times.get("fb1_ms", 0.0)
                        + phase_times.get("fb2_ms", 0.0)
                    ),
                    "validation_total_ms": float(
                        phase_times.get("candidate_gen_ms", 0.0)
                        + phase_times.get("detect_ms", 0.0)
                    ),
                    "message": last_result.message,
                }
            )
            last_result.pass_history = list(pass_history)

            if not last_result.success:
                return last_result

        remaining_conflicts = self._rtree_detector.detect_all_conflicts(target_uav_id)
        if remaining_conflicts:
            capped_result = ResolveResult(
                success=False,
                strategy_used="DEADLOCK",
                message=(
                    f"Safety cap reached before '{target_uav_id}' became conflict-free. "
                    f"Remaining conflicts: {len(remaining_conflicts)}"
                ),
            )
            capped_result.pass_history = list(pass_history)
            logger.error(
                f"[CentralManager] Safety cap reached while resolving '{target_uav_id}'. "
                f"Remaining conflicts: {len(remaining_conflicts)}"
            )
            return capped_result

        return last_result

    # ------------------------------------------------------------------
    # Convenience: system-wide sweep
    # ------------------------------------------------------------------

    def single_solve_sweep(self, interval: float = OBB_INTERVAL) -> Dict[str, ResolveResult]:
        """
        Run check_and_resolve for every registered UAV in priority order
        (lowest priority first, so VIPs are never the target of modification).

        Returns a dict: {uav_id: ResolveResult | None}
        Only UAVs for which a conflict was detected will have a non-None entry.
        """
        # Sort by ascending priority so low-priority UAVs are checked first
        ordered_ids = sorted(
            self._flight_plans.keys(),
            key=lambda uid: self._priorities.get(uid, 0),
        )

        results: Dict[str, Optional[ResolveResult]] = {}
        for uav_id in ordered_ids:
            result = self.check_and_resolve(uav_id, interval=interval)
            if result is not None:
                results[uav_id] = result

        return results

    def solve_all(self, max_sweeps: int = 100, interval: float = OBB_INTERVAL) -> dict:
        """
        Iteratively resolve system-wide conflicts until the airspace stabilizes.

        The manager performs repeated global sweeps. Each sweep checks whether
        any conflicts remain, then runs a priority-ordered resolution pass. The
        loop stops when the system is clear, when a sweep makes no progress, or
        when the sweep limit is reached.
        """
        import time

        summary = {
            "sweeps": 0,
            "attempted": 0,
            "resolved": 0,
            "deadlocks": 0,
            "anchor_deadlocks": 0,
            "hover_deadlocks": 0,
            "remaining_conflicts": 0,
            "stabilized": False,
        }

        t0 = time.perf_counter()

        for sweep_idx in range(1, max_sweeps + 1):
            conflicts = self._rtree_detector.detect_all_conflicts_system_wide()
            if not conflicts:
                summary["stabilized"] = True
                print(
                    f"[CentralManager] Sweep {sweep_idx}: airspace already conflict-free; stopping."
                )
                break

            summary["sweeps"] += 1
            print(
                f"[CentralManager] Sweep {sweep_idx}/{max_sweeps}: "
                f"conflicts_before={len(conflicts)}"
            )
            sweep_results = self.single_solve_sweep(interval=interval)

            if not sweep_results:
                print(
                    f"[CentralManager] Sweep {sweep_idx}: no UAVs produced a resolution result; stopping."
                )
                break

            resolved_this_sweep = 0
            for result in sweep_results.values():
                summary["attempted"] += 1
                if result.success:
                    summary["resolved"] += 1
                    resolved_this_sweep += 1
                else:
                    summary["deadlocks"] += 1
                    deadlock_type = str(getattr(result, "deadlock_type", "unknown"))
                    if deadlock_type in {"anchor_before_conflict", "anchor_after_finish"}:
                        summary["anchor_deadlocks"] += 1
                    elif deadlock_type == "hover_timeout":
                        summary["hover_deadlocks"] += 1

            print(
                f"[CentralManager] Sweep {sweep_idx}: "
                f"attempted={len(sweep_results)} resolved={resolved_this_sweep} "
                f"deadlocks={len(sweep_results) - resolved_this_sweep}"
            )

            if resolved_this_sweep == 0:
                print(
                    f"[CentralManager] Sweep {sweep_idx}: no progress made; stopping."
                )
                break

        summary["remaining_conflicts"] = len(self._rtree_detector.detect_all_conflicts_system_wide())
        summary["stabilized"] = summary["remaining_conflicts"] == 0
        summary["time_s"] = time.perf_counter() - t0
        summary["max_sweeps"] = max_sweeps
        summary["interval"] = interval

        print(
            f"[CentralManager] Sweep loop finished: sweeps={summary['sweeps']} "
            f"remaining_conflicts={summary['remaining_conflicts']} stabilized={summary['stabilized']}"
        )
        return summary
