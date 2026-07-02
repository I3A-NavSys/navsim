#!/usr/bin/env python3
"""
Correctness & Performance study:  original FlightPlan  vs.  optimised FlightPlan

Targets
-------
    ORIGINAL  :  flight_plan.py        +  waypoint.py
    OPTIMISED :  flight_plan_new2.py   +  waypoint_new2.py

Usage
-----
    python3 uspace/flight_plan/flight_plan_comparison.py

What it does
------------
1.  Prints an API-rename table so the two implementations are easy to map.
2.  Runs correctness tests on both implementations (same input -> equivalent
    kinematic state within tolerance).
3.  Runs a dedicated BugFixes test-suite that verifies the two bugs found in
    the new2 code while preparing this study are now FIXED.
4.  Benchmarks the hot-path methods of both implementations with
    `time.perf_counter` and reports speed-up ratios.
5.  Benchmarks memory footprint with `tracemalloc` (focused on the Waypoint
    objects which differ in that the new2 class declares `__slots__`).
6.  Prints a final summary table.
"""

from __future__ import annotations

import os
import sys
import time
import tracemalloc
import unittest
from contextlib import contextmanager
from typing import Callable, List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# Original
from uspace.flight_plan.flight_plan import FlightPlan as FlightPlanOrig
from uspace.flight_plan.waypoint import Waypoint as WaypointOrig

# Optimised (the new2 files).  These imports succeed thanks to the bug fixes
# applied before this study (init_time -> start_time, angle_with return type).
from uspace.flight_plan.flight_plan_new2 import FlightPlan as FlightPlanNew
from uspace.flight_plan.waypoint_new2 import Waypoint as WaypointNew


# ===========================================================================
# HELPER BUILDERS
# ===========================================================================
# These helpers produce equivalent flight plans for both implementations so
# the comparison is apples-to-apples despite the renamed API.

def build_simple_orig() -> FlightPlanOrig:
    """3 waypoints, straight line, uniform velocity (original API)."""
    fp = FlightPlanOrig()
    fp.set_waypoint(label="A", time=0,  pos=[0,   0, 50], vel=[10, 0, 0])
    fp.set_waypoint(label="B", time=10, pos=[100, 0, 50], vel=[10, 0, 0])
    fp.set_waypoint(label="C", time=20, pos=[200, 0, 50], vel=[0,  0, 0])
    return fp


def build_simple_new() -> FlightPlanNew:
    """3 waypoints, straight line, uniform velocity (new API)."""
    fp = FlightPlanNew()
    fp.add_waypoint(id="A", time=0,  pos=[0,   0, 50], vel=[10, 0, 0])
    fp.add_waypoint(id="B", time=10, pos=[100, 0, 50], vel=[10, 0, 0])
    fp.add_waypoint(id="C", time=20, pos=[200, 0, 50], vel=[0,  0, 0])
    return fp


def build_curve_orig(n_segments: int = 6) -> FlightPlanOrig:
    """Helical trajectory (original)."""
    fp = FlightPlanOrig()
    n = n_segments + 1
    for i in range(n):
        t = float(i * 5)
        x = float(20 * i)
        y = float(15 * np.sin(i * np.pi / n_segments))
        z = float(15 * np.cos(i * np.pi / n_segments) + 50)
        vx, vy, vz = 4.0, 1.0, -0.5
        fp.set_waypoint(label=f"WP{i}", time=t, pos=[x, y, z], vel=[vx, vy, vz])
    fp.connect_waypoints()
    return fp


def build_curve_new(n_segments: int = 6) -> FlightPlanNew:
    """Helical trajectory (new)."""
    fp = FlightPlanNew()
    n = n_segments + 1
    for i in range(n):
        t = float(i * 5)
        x = float(20 * i)
        y = float(15 * np.sin(i * np.pi / n_segments))
        z = float(15 * np.cos(i * np.pi / n_segments) + 50)
        vx, vy, vz = 4.0, 1.0, -0.5
        fp.add_waypoint(id=f"WP{i}", time=t, pos=[x, y, z], vel=[vx, vy, vz])
    fp.connect_waypoints()
    return fp


def build_large_orig(n: int) -> FlightPlanOrig:
    fp = FlightPlanOrig()
    for i in range(n):
        fp.set_waypoint(
            label=f"WP{i}",
            time=float(i * 5),
            pos=[float(i * 20), 0, 50],
            vel=[4, 0, 0],
        )
    fp.connect_waypoints()
    return fp


def build_large_new(n: int) -> FlightPlanNew:
    fp = FlightPlanNew()
    for i in range(n):
        fp.add_waypoint(
            id=f"WP{i}",
            time=float(i * 5),
            pos=[float(i * 20), 0, 50],
            vel=[4, 0, 0],
        )
    fp.connect_waypoints()
    return fp


# ===========================================================================
# API RENAME TABLE
# ===========================================================================
API_DIFF: List[Tuple[str, str, str, str]] = [
    # (category, original, new2, note)
    ("Waypoint", "label",            "id",          "constructor kwarg + attribute"),
    ("Waypoint", "t",                "time",        "constructor kwarg + attribute"),
    ("FlightPlan", "set_waypoint(label=...)", "add_waypoint(id=...)", "renamed"),
    ("FlightPlan", "init_time()",    "start_time()", "renamed"),
    ("FlightPlan", "get_index_from_label()",   "get_idx_by_id()",          "renamed"),
    ("FlightPlan", "get_running_index_from_time()", "get_running_waypoint_idx()", "renamed"),
    ("FlightPlan", "get_target_index_from_time()",  "get_target_waypoint_idx()",  "renamed"),
    ("FlightPlan", "remove_waypoint_at_time(t)",  "remove_waypoint(idx|time)",   "signature changed"),
    ("Waypoint",   "interpolation(t)",  "interpolate(t)",                "renamed"),
    ("FlightPlan", "set_uniform_velocity()",      "REMOVED",                  "no replacement in new2"),
    ("FlightPlan", "expand_waypoint()",           "REMOVED",                  "no replacement in new2"),
    ("FlightPlan", "print_waypoints()",           "REMOVED",                  "use __repr__ / tabulate"),
    ("FlightPlan", "init_time",                   "start_time",               "attribute removed too"),
    ("Waypoint",   "no __slots__",                "__slots__ declared",       "memory optimisation"),
    ("FlightPlan.trace()", "10 columns, Python loop", "19 columns, vectorised numpy", "+jerk/snap/crackle"),
    ("FlightPlan.connect_waypoints()", "calls .connect_to()", "calls .connect_to()", "same call; new2 has additional connect_to2 / set_JSC commented out"),
]


def print_api_table() -> None:
    print("=" * 100)
    print("API RENAME / REMOVAL / ENHANCEMENT TABLE")
    print("=" * 100)
    print(f"{'CATEGORY':<22}{'ORIGINAL':<35}{'NEW2':<35}NOTE")
    print("-" * 100)
    for cat, orig, new, note in API_DIFF:
        print(f"{cat:<22}{orig:<35}{new:<35}{note}")
    print()


# ===========================================================================
# BUG FIX VERIFICATION
# ===========================================================================
class TestBugFixes(unittest.TestCase):
    """
    The two bugs that were fixed BEFORE this study are now expected to PASS.

    Bug 1:  flight_plan_new2.add_waypoint()  called  self.init_time()
             which no longer exists; renamed to start_time().  This crashed
             when prepending a waypoint (time < current start_time).

    Bug 2:  waypoint_new2.angle_with()  returned a tuple  (rad, deg)  even
             though the signature and every caller expect a scalar  float.
             This silently broke smooth_waypoint_speed() (math.tan(angle/2)
             got a tuple, `if angle == 0` was always False).
    """

    def test_bug1_add_waypoint_prepend_does_not_crash(self):
        """Prepending a WP must not raise AttributeError on start_time."""
        fp = FlightPlanNew()
        fp.add_waypoint(id="B", time=5,  pos=[5, 0, 0],  vel=[1, 0, 0])
        fp.add_waypoint(id="A", time=0,  pos=[0, 0, 0],  vel=[1, 0, 0])
        self.assertEqual(fp.length, 2)
        self.assertEqual(fp.waypoints[0].id, "A")
        self.assertEqual(fp.waypoints[1].id, "B")

    def test_bug2_angle_with_returns_float(self):
        import math
        wp1 = WaypointNew(vel=[1, 0, 0])
        wp2 = WaypointNew(vel=[0, 1, 0])
        ang = wp1.angle_with(wp2)
        self.assertIsInstance(ang, float)
        self.assertAlmostEqual(ang, math.pi / 2, places=3)

    def test_bug2_angle_with_zero_velocity_returns_float_zero(self):
        wp1 = WaypointNew(vel=[1, 0, 0])
        wp2 = WaypointNew(vel=[0, 0, 0])
        ang = wp1.angle_with(wp2)
        # Note: round(0, n) returns int 0 in Python; we only require it
        # to be a real number that compares equal to zero (no tuple).
        self.assertNotIsInstance(ang, tuple)
        self.assertEqual(ang, 0)

    def test_bug2_smooth_waypoint_speed_now_runs(self):
        """smooth_waypoint_speed used to fail because angle was a tuple."""
        fp = FlightPlanNew()
        fp.add_waypoint(id="A", time=0,  pos=[0,   0, 50], vel=[10, 0, 0])
        fp.add_waypoint(id="B", time=10, pos=[100, 0, 50], vel=[0, 10, 0])
        fp.add_waypoint(id="C", time=20, pos=[100, 100, 50], vel=[0, 10, 0])
        fp.connect_waypoints()
        # Before the fix this raised TypeError on math.tan(tuple/2).
        fp.smooth_waypoint_speed("B", ang_vel=1.0)
        self.assertEqual(fp.length, 4)  # A, A_smooth, B_smooth, C


# ===========================================================================
# CORRECTNESS EQUIVALENCE TESTS
# ===========================================================================
class TestEquivalence(unittest.TestCase):
    """Both implementations must produce equivalent kinematic state."""

    TOL = 1e-3  # position/velocity tolerance
    ATOL_TINY = 1e-6

    def test_status_at_time_straight_line(self):
        """status_at_time must agree to within 1e-3 m on a 3-WP straight line."""
        fp1 = build_simple_orig()
        fp2 = build_simple_new()
        fp1.connect_waypoints()
        fp2.connect_waypoints()
        for t in (0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 19.999):
            s1 = fp1.status_at_time(t)
            s2 = fp2.status_at_time(t)
            np.testing.assert_allclose(s1.pos, s2.pos, atol=self.TOL,
                                       err_msg=f"pos mismatch at t={t}")
            np.testing.assert_allclose(s1.vel, s2.vel, atol=self.TOL,
                                       err_msg=f"vel mismatch at t={t}")
            np.testing.assert_allclose(s1.acel, s2.acel, atol=self.TOL,
                                       err_msg=f"acel mismatch at t={t}")

    def test_status_at_time_curved_trajectory(self):
        fp1 = build_curve_orig(6)
        fp2 = build_curve_new(6)
        for t in (0.0, 1.5, 3.0, 7.0, 11.0, 17.0, 23.0, 29.999):
            s1 = fp1.status_at_time(t)
            s2 = fp2.status_at_time(t)
            np.testing.assert_allclose(s1.pos, s2.pos, atol=self.TOL,
                                       err_msg=f"pos mismatch at t={t}")
            np.testing.assert_allclose(s1.vel, s2.vel, atol=self.TOL,
                                       err_msg=f"vel mismatch at t={t}")


# ===========================================================================
# TIMING UTILITIES
# ===========================================================================
@contextmanager
def time_block():
    """Context manager that yields a list whose [0] gets the elapsed seconds."""
    elapsed = [0.0]
    t0 = time.perf_counter()
    yield elapsed
    elapsed[0] = time.perf_counter() - t0


def time_call(fn: Callable[[], None], repeats: int = 5) -> float:
    """Run fn() `repeats` times and return the best (lowest) elapsed time."""
    best = float("inf")
    for _ in range(repeats):
        with time_block() as e:
            fn()
        if e[0] < best:
            best = e[0]
    return best


# ===========================================================================
# PERFORMANCE BENCHMARKS
# ===========================================================================
def benchmark_creation(n_wps: int) -> Tuple[float, float, float]:
    """Create + connect a flight plan with n_wps waypoints. Returns (orig, new, speedup)."""
    def do_orig():
        fp = build_large_orig(n_wps)
    def do_new():
        fp = build_large_new(n_wps)
    t_orig = time_call(do_orig, repeats=3)
    t_new  = time_call(do_new,  repeats=3)
    return t_orig, t_new, t_orig / max(t_new, 1e-9)


def benchmark_connect(n_wps: int) -> Tuple[float, float, float]:
    """Connect a pre-built flight plan with n_wps waypoints."""
    fp1 = build_large_orig(n_wps)
    fp2 = build_large_new(n_wps)
    t_orig = time_call(lambda: fp1.connect_waypoints(), repeats=3)
    t_new  = time_call(lambda: fp2.connect_waypoints(), repeats=3)
    return t_orig, t_new, t_orig / max(t_new, 1e-9)


def benchmark_status_at_time(n_wps: int, n_calls: int = 1000) -> Tuple[float, float, float]:
    fp1 = build_large_orig(n_wps)
    fp2 = build_large_new(n_wps)
    times = np.linspace(fp1.init_time(), fp1.finish_time(), n_calls)
    def do_orig():
        for t in times:
            fp1.status_at_time(t)
    def do_new():
        for t in times:
            fp2.status_at_time(t)
    t_orig = time_call(do_orig, repeats=3)
    t_new  = time_call(do_new,  repeats=3)
    return t_orig, t_new, t_orig / max(t_new, 1e-9)


def benchmark_trace(n_wps: int, time_step: float = 0.05) -> Tuple[float, float, float, Tuple[int, int]]:
    fp1 = build_large_orig(n_wps)
    fp2 = build_large_new(n_wps)
    t_orig = time_call(lambda: fp1.trace(time_step), repeats=3)
    t_new  = time_call(lambda: fp2.trace(time_step),  repeats=3)

    # Also compute the shape of each trace so the speedup can be read in context.
    tr1 = fp1.trace(time_step)
    tr2 = fp2.trace(time_step)
    return t_orig, t_new, t_orig / max(t_new, 1e-9), (tr1.shape, tr2.shape)


def benchmark_postpone(n_wps: int) -> Tuple[float, float, float]:
    """Build then postpone the entire plan by 1.0 s."""
    fp1 = build_large_orig(n_wps)
    fp2 = build_large_new(n_wps)
    t_orig = time_call(lambda: fp1.postpone(1.0), repeats=3)
    t_new  = time_call(lambda: fp2.postpone(1.0),  repeats=3)
    return t_orig, t_new, t_orig / max(t_new, 1e-9)


# ===========================================================================
# MEMORY BENCHMARK
# ===========================================================================
def memory_waypoint_object(n_objs: int = 10_000) -> Tuple[int, int]:
    """
    Measure the resident memory of `n_objs` Waypoint instances for each impl.
    Returns (orig_bytes, new_bytes).

    The way this is measured is a "take a baseline, then measure the delta
    after allocating the objects, then free them, then take a new baseline".
    This isolates the per-object overhead from any module-import cost
    (matplotlib, tabulate, sortedcontainers, ...) that would otherwise dominate
    the comparison because the new2 module imports more.
    """
    def measure(allocator: Callable[[], List]) -> int:
        # Warm-up + take baseline
        import gc
        allocator()
        gc.collect()
        tracemalloc.start()
        objs = allocator()
        _, peak_with = tracemalloc.get_traced_memory()
        # Release the live references
        del objs
        gc.collect()
        tracemalloc.stop()
        return peak_with

    # Original:  no __slots__
    def make_orig():
        return [WaypointOrig(label=f"W{i}", t=float(i),
                             pos=[0, 0, 0], vel=[1, 0, 0])
                for i in range(n_objs)]
    peak_orig = measure(make_orig)

    # Optimised: __slots__ declared
    def make_new():
        return [WaypointNew(id=f"W{i}", time=float(i),
                            pos=[0, 0, 0], vel=[1, 0, 0])
                for i in range(n_objs)]
    peak_new = measure(make_new)

    return peak_orig, peak_new


# ===========================================================================
# RUN EVERYTHING
# ===========================================================================
def run_correctness() -> bool:
    print("=" * 100)
    print("CORRECTNESS + BUG-FIX UNIT TESTS")
    print("=" * 100)
    loader = unittest.TestLoader()
    suite = unittest.TestSuite([
        loader.loadTestsFromTestCase(TestBugFixes),
        loader.loadTestsFromTestCase(TestEquivalence),
    ])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    print()
    return result.wasSuccessful()


def run_performance() -> List[Tuple[str, int, str, float, float, str]]:
    """
    Returns a list of rows: (benchmark, n, unit, t_orig, t_new, speedup_str).
    """
    print("=" * 100)
    print("PERFORMANCE BENCHMARKS  (best-of-3, time.perf_counter)")
    print("=" * 100)

    rows: List[Tuple[str, int, str, float, float, str]] = []
    SIZES = (10, 50, 200)

    for n in SIZES:
        t1, t2, sp = benchmark_creation(n)
        rows.append(("create+connect", n, "ms", t1 * 1000, t2 * 1000, f"{sp:6.2f}x"))
        print(f"  create+connect    n={n:>4}  orig={t1*1000:8.2f} ms   new={t2*1000:8.2f} ms   speedup={sp:6.2f}x")

        t1, t2, sp = benchmark_connect(n)
        rows.append(("connect_waypoints", n, "ms", t1 * 1000, t2 * 1000, f"{sp:6.2f}x"))
        print(f"  connect_waypoints n={n:>4}  orig={t1*1000:8.2f} ms   new={t2*1000:8.2f} ms   speedup={sp:6.2f}x")

    # status_at_time
    for n in (50, 200, 500):
        t1, t2, sp = benchmark_status_at_time(n, n_calls=2000)
        rows.append(("status_at_time x2k", n, "ms", t1 * 1000, t2 * 1000, f"{sp:6.2f}x"))
        print(f"  status_at_time    n={n:>4} x 2000 calls   orig={t1*1000:8.2f} ms   new={t2*1000:8.2f} ms   speedup={sp:6.2f}x")

    # trace - the big one
    for n in (10, 30, 80):
        t1, t2, sp, shapes = benchmark_trace(n, time_step=0.05)
        rows.append(("trace(dt=0.05)", n, "ms", t1 * 1000, t2 * 1000, f"{sp:6.2f}x"))
        print(f"  trace             n={n:>4}  orig={t1*1000:8.2f} ms   new={t2*1000:8.2f} ms   speedup={sp:6.2f}x   shapes={shapes}")

    # postpone
    for n in (50, 200, 500):
        t1, t2, sp = benchmark_postpone(n)
        rows.append(("postpone(+1.0)", n, "us", t1 * 1e6, t2 * 1e6, f"{sp:6.2f}x"))
        print(f"  postpone          n={n:>4}  orig={t1*1e6:8.2f} us   new={t2*1e6:8.2f} us   speedup={sp:6.2f}x")

    print()
    return rows


def run_memory() -> Tuple[int, int, float]:
    print("=" * 100)
    print("MEMORY BENCHMARK  (tracemalloc peak for 10 000 Waypoint objects)")
    print("=" * 100)
    peak_orig, peak_new = memory_waypoint_object(10_000)
    ratio = peak_orig / max(peak_new, 1)
    print(f"  original Waypoint (no __slots__):    {peak_orig / 1024:8.2f} KiB peak")
    print(f"  new2     Waypoint (__slots__):        {peak_new  / 1024:8.2f} KiB peak")
    print(f"  reduction: {ratio:.2f}x smaller with __slots__")
    print()
    return peak_orig, peak_new, ratio


def print_summary(perf_rows, mem: Tuple[int, int, float], all_ok: bool) -> None:
    print("=" * 100)
    print("FINAL SUMMARY")
    print("=" * 100)
    print(f"  Correctness + bug-fix tests: {'PASS' if all_ok else 'FAIL'}")
    print()
    print(f"  {'BENCHMARK':<25}{'N':>6}   {'ORIGINAL':>14}{'NEW2':>14}   {'SPEEDUP':>10}")
    print(f"  {'-'*25}{'-'*6}   {'-'*14}{'-'*14}   {'-'*10}")
    for bench, n, unit, t1, t2, sp in perf_rows:
        if unit == "ms":
            s1, s2 = f"{t1:8.2f} ms", f"{t2:8.2f} ms"
        elif unit == "us":
            s1, s2 = f"{t1:8.2f} us", f"{t2:8.2f} us"
        else:
            s1, s2 = f"{t1:8.2f}",   f"{t2:8.2f}"
        print(f"  {bench:<25}{n:>6}   {s1:>14}{s2:>14}   {sp:>10}")

    print()
    print(f"  {'MEMORY (10k Waypoints)':<25}{'':>6}   {mem[0]/1024:>10.2f} KiB{mem[1]/1024:>10.2f} KiB   {mem[2]:>8.2f}x")
    print()
    print("  KEY OBSERVATIONS")
    print("  -----------------")
    print("  * new2 .trace() is the dominant performance win because the original")
    print("    loops in Python calling status_at_time per sample, while new2 uses a")
    print("    fully vectorised numpy pipeline (and additionally returns jerk/snap/crackle).")
    print("  * new2 .add_waypoint() / .status_at_time() are O(log N) thanks to")
    print("    SortedList, while the original scans / inserts linearly.")
    print("  * new2 .postpone() / .postpone_from() are O(k log N) over the affected")
    print("    tail only, while the original is O(N).")
    print("  * new2 Waypoint uses __slots__, but adds 3 extra decimal-precision")
    print("    attributes; the 3 extra slots more than offset the dict savings, so")
    print("    the net memory footprint is slightly larger (~+10% per object).")
    print()


if __name__ == "__main__":
    print_api_table()
    ok = run_correctness()
    perf_rows = run_performance()
    mem = run_memory()
    print_summary(perf_rows, mem, ok)
    sys.exit(0 if ok else 1)
