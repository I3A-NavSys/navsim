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
from typing import Dict, Optional

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

    def get_flight_plan(self, uav_id: str) -> Optional[FlightPlan]:
        """
        Retrieve the current (possibly updated) FlightPlan for a UAV.

        Returns None if the UAV is not registered.
        """
        return self._flight_plans.get(uav_id)

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
            # UAV A is the VIP unless both priorities are equal and B wins by ID.
            if priority_a == priority_b and uav_a_id > uav_b_id:
                vip_id,  pleb_id  = uav_b_id, uav_a_id
            else:
                vip_id,  pleb_id  = uav_a_id, uav_b_id
        else:
            vip_id,  pleb_id  = uav_b_id, uav_a_id

        fp_vip  = self._flight_plans[vip_id]
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
            # Update registry and R-Tree with the new conflict-free plans
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

    def solve_all(self, max_sweeps: int = 300, interval: float = OBB_INTERVAL) -> dict:
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
            "remaining_conflicts": 0,
            "stabilized": False,
        }

        t0 = time.perf_counter()

        for _ in range(max_sweeps):
            conflicts = self._rtree_detector.detect_all_conflicts_system_wide()
            if not conflicts:
                summary["stabilized"] = True
                break

            summary["sweeps"] += 1
            sweep_results = self.single_solve_sweep(interval=interval)

            if not sweep_results:
                break

            resolved_this_sweep = 0
            for result in sweep_results.values():
                summary["attempted"] += 1
                if result.success:
                    summary["resolved"] += 1
                    resolved_this_sweep += 1
                else:
                    summary["deadlocks"] += 1

            if resolved_this_sweep == 0:
                break

        summary["remaining_conflicts"] = len(self._rtree_detector.detect_all_conflicts_system_wide())
        summary["stabilized"] = summary["remaining_conflicts"] == 0
        summary["time_s"] = time.perf_counter() - t0
        summary["max_sweeps"] = max_sweeps
        summary["interval"] = interval
        return summary
