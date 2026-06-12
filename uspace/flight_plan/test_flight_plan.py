#!/usr/bin/env python3
"""
Correctness & Performance test suite for the FlightPlan class.

Run from the repository root:
    python uspace/flight_plan/test_flight_plan.py

Or from any directory:
    python /path/to/navsim/uspace/flight_plan/test_flight_plan.py

Sections
--------
1. Helper builders
2. Unit / correctness tests  (unittest.TestCase)
3. Performance benchmarks    (time.perf_counter)
4. Complexity notes & bug report
"""

import sys
import os
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# Path setup — ensures imports work regardless of cwd
# ---------------------------------------------------------------------------
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# from uspace.flight_plan.flight_plan import FlightPlan
# from uspace.flight_plan.waypoint import Waypoint
from uspace.flight_plan.flight_plan_new import FlightPlan
from uspace.flight_plan.waypoint_new import Waypoint
from uspace.flight_plan.command import Command


# ===========================================================================
# HELPER BUILDERS
# ===========================================================================

def build_simple_fp() -> FlightPlan:
    """3-waypoint straight flight plan with pre-set velocities."""
    fp = FlightPlan()
    fp.set_waypoint(label="A", time=0,  pos=[0,   0, 50], vel=[10, 0, 0])
    fp.set_waypoint(label="B", time=10, pos=[100, 0, 50], vel=[10, 0, 0])
    fp.set_waypoint(label="C", time=20, pos=[200, 0, 50], vel=[0,  0, 0])
    return fp


def build_connected_fp(n_segments: int = 5) -> FlightPlan:
    """Flight plan with sinusoidal trajectory, fully connected (jerk/snap computed)."""
    fp = FlightPlan()
    for i in range(n_segments + 1):
        t   = float(i * 10)
        x   = float(i * 100)
        y   = float(np.sin(i * np.pi / n_segments) * 50)
        z   = float(i * 5 + 50)
        vx  = 10.0
        vy  = float(np.cos(i * np.pi / n_segments) * 5)
        vz  = 0.5
        fp.set_waypoint(label=f"WP{i}", time=t, pos=[x, y, z], vel=[vx, vy, vz])
    fp.connect_waypoints()
    return fp


def build_large_fp(n: int) -> FlightPlan:
    """Flight plan with n waypoints in a straight line at uniform speed."""
    fp = FlightPlan()
    for i in range(n):
        fp.set_waypoint(
            label=f"WP{i}",
            time=float(i * 10),
            pos=[float(i * 100), 0, 50],
            vel=[10, 0, 0],
        )
    fp.connect_waypoints()
    return fp


# ===========================================================================
# UNIT / CORRECTNESS TESTS
# ===========================================================================

class TestFlightPlanBasic(unittest.TestCase):
    """Default state and __repr__."""

    def test_default_attributes(self):
        fp = FlightPlan()
        self.assertEqual(fp.id, 0)
        self.assertEqual(fp.priority, 0)
        self.assertAlmostEqual(fp.radius, 1.0)
        self.assertEqual(fp.waypoints, [])
        self.assertIsNone(fp.target_yaw)

    def test_repr_contains_class_name(self):
        fp = FlightPlan()
        self.assertIn("FlightPlan", repr(fp))

    def test_repr_shows_waypoint_count(self):
        fp = build_simple_fp()
        self.assertIn("3", repr(fp))


class TestWaypointInsertion(unittest.TestCase):
    """set_waypoint: ordering, replacement, auto-values."""

    def test_insert_in_order_preserves_order(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=0)
        fp.set_waypoint(label="B", time=10)
        fp.set_waypoint(label="C", time=20)
        times = [wp.t for wp in fp.waypoints]
        self.assertEqual(times, sorted(times))

    def test_insert_out_of_order_is_sorted(self):
        fp = FlightPlan()
        fp.set_waypoint(label="C", time=20)
        fp.set_waypoint(label="A", time=0)
        fp.set_waypoint(label="B", time=10)
        labels = [wp.label for wp in fp.waypoints]
        self.assertEqual(labels, ["A", "B", "C"])

    def test_insert_between_existing_waypoints(self):
        fp = build_simple_fp()           # t = 0, 10, 20
        fp.set_waypoint(label="MID", time=5)
        times = [wp.t for wp in fp.waypoints]
        self.assertIn(5, times)
        self.assertEqual(times, sorted(times))

    def test_replace_at_same_time(self):
        fp = FlightPlan()
        fp.set_waypoint(label="old", time=5, pos=[0, 0, 0])
        fp.set_waypoint(label="new", time=5, pos=[1, 1, 1])
        self.assertEqual(len(fp.waypoints), 1)
        self.assertEqual(fp.waypoints[0].label, "new")
        np.testing.assert_array_equal(fp.waypoints[0].pos, [1, 1, 1])

    def test_auto_time_is_finish_plus_one(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=5)
        fp.set_waypoint(label="B")          # should default to t = 6
        self.assertAlmostEqual(fp.waypoints[1].t, 6.0)

    def test_first_waypoint_auto_time_is_zero(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A")
        self.assertAlmostEqual(fp.waypoints[0].t, 0.0)

    def test_first_waypoint_auto_pos_is_origin(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A")
        np.testing.assert_array_equal(fp.waypoints[0].pos, [0, 0, 0])


class TestWaypointRemoval(unittest.TestCase):
    """remove_waypoint_at_time."""

    def test_remove_existing_waypoint(self):
        fp = build_simple_fp()
        fp.remove_waypoint_at_time(10)
        times = [wp.t for wp in fp.waypoints]
        self.assertNotIn(10, times)
        self.assertEqual(len(fp.waypoints), 2)

    def test_remove_nonexistent_is_noop(self):
        fp = build_simple_fp()
        fp.remove_waypoint_at_time(999)
        self.assertEqual(len(fp.waypoints), 3)

    def test_remove_first_waypoint(self):
        fp = build_simple_fp()
        fp.remove_waypoint_at_time(0)
        self.assertAlmostEqual(fp.waypoints[0].t, 10)

    def test_remove_last_waypoint(self):
        fp = build_simple_fp()
        fp.remove_waypoint_at_time(20)
        self.assertAlmostEqual(fp.finish_time(), 10)


class TestIndexLookup(unittest.TestCase):
    """get_target_index_from_time / get_running_index_from_time / get_index_from_label."""

    def setUp(self):
        self.fp = build_simple_fp()   # t = 0, 10, 20

    def test_target_index_before_first_wp(self):
        self.assertEqual(self.fp.get_target_index_from_time(-5), 0)

    def test_target_index_at_first_wp(self):
        # t == wp.t  → that WP is considered target
        self.assertEqual(self.fp.get_target_index_from_time(0), 0)

    def test_target_index_between_first_and_second(self):
        # flying towards index 1
        self.assertEqual(self.fp.get_target_index_from_time(5), 1)

    def test_target_index_at_middle_wp(self):
        self.assertEqual(self.fp.get_target_index_from_time(10), 1)

    def test_target_index_after_last_wp(self):
        idx = self.fp.get_target_index_from_time(100)
        self.assertEqual(idx, len(self.fp.waypoints))

    def test_running_index_at_start(self):
        self.assertEqual(self.fp.get_running_index_from_time(0), 0)

    def test_running_index_between_first_and_second(self):
        self.assertEqual(self.fp.get_running_index_from_time(5), 0)

    def test_running_index_between_second_and_third(self):
        self.assertEqual(self.fp.get_running_index_from_time(15), 1)

    def test_get_index_from_label_found(self):
        self.assertEqual(self.fp.get_index_from_label("B"), 1)

    def test_get_index_from_label_not_found(self):
        self.assertIsNone(self.fp.get_index_from_label("NONEXISTENT"))

    def test_get_running_waypoint_returns_waypoint(self):
        wp = self.fp.get_running_waypoint(5)
        self.assertIsInstance(wp, Waypoint)

    def test_empty_fp_target_index_zero(self):
        fp = FlightPlan()
        self.assertEqual(fp.get_target_index_from_time(5), 0)


class TestTimeManagement(unittest.TestCase):
    """init_time, finish_time, postpone, postpone_from, reschedule_at, remove_negative_time."""

    def test_init_time(self):
        fp = build_simple_fp()
        self.assertAlmostEqual(fp.init_time(), 0)

    def test_finish_time(self):
        fp = build_simple_fp()
        self.assertAlmostEqual(fp.finish_time(), 20)

    def test_empty_fp_init_time_is_zero(self):
        fp = FlightPlan()
        self.assertAlmostEqual(fp.init_time(), 0)

    def test_empty_fp_finish_time_is_zero(self):
        fp = FlightPlan()
        self.assertAlmostEqual(fp.finish_time(), 0)

    def test_postpone_shifts_all_waypoints(self):
        fp = build_simple_fp()
        fp.postpone(100)
        self.assertAlmostEqual(fp.init_time(), 100)
        self.assertAlmostEqual(fp.finish_time(), 120)

    def test_postpone_preserves_intervals(self):
        fp = build_simple_fp()
        original_intervals = [fp.waypoints[i+1].t - fp.waypoints[i].t
                               for i in range(len(fp.waypoints) - 1)]
        fp.postpone(50)
        new_intervals = [fp.waypoints[i+1].t - fp.waypoints[i].t
                         for i in range(len(fp.waypoints) - 1)]
        for o, n in zip(original_intervals, new_intervals):
            self.assertAlmostEqual(o, n, places=5)

    def test_postpone_from_only_shifts_later_waypoints(self):
        fp = build_simple_fp()          # t = 0, 10, 20
        fp.postpone_from(10, 5)         # shift from t=10 onwards by +5
        times = [wp.t for wp in fp.waypoints]
        self.assertAlmostEqual(times[0], 0)
        self.assertAlmostEqual(times[1], 15)
        self.assertAlmostEqual(times[2], 25)

    def test_postpone_from_past_end_is_noop(self):
        fp = build_simple_fp()
        fp.postpone_from(999, 10)
        self.assertAlmostEqual(fp.finish_time(), 20)

    def test_reschedule_at(self):
        fp = build_simple_fp()
        fp.reschedule_at(50)
        self.assertAlmostEqual(fp.init_time(), 50)
        self.assertAlmostEqual(fp.finish_time(), 70)

    def test_remove_negative_time_makes_init_nonnegative(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=-10, pos=[0, 0, 0])
        fp.set_waypoint(label="B", time=5,   pos=[10, 0, 0])
        fp.remove_negative_time()
        self.assertGreaterEqual(fp.init_time(), 0)

    def test_remove_negative_time_preserves_interval(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=-10, pos=[0, 0, 0])
        fp.set_waypoint(label="B", time=5,   pos=[10, 0, 0])
        interval_before = fp.waypoints[1].t - fp.waypoints[0].t
        fp.remove_negative_time()
        interval_after = fp.waypoints[1].t - fp.waypoints[0].t
        self.assertAlmostEqual(interval_before, interval_after, places=5)


class TestStatusAtTime(unittest.TestCase):
    """status_at_time boundary conditions and interpolation quality."""

    def setUp(self):
        self.fp = build_connected_fp(n_segments=4)

    def test_status_at_init_time_matches_first_wp(self):
        status = self.fp.status_at_time(self.fp.init_time())
        np.testing.assert_array_almost_equal(
            status.pos, self.fp.waypoints[0].pos, decimal=5
        )

    def test_status_at_finish_time_matches_last_wp(self):
        status = self.fp.status_at_time(self.fp.finish_time())
        np.testing.assert_array_almost_equal(
            status.pos, self.fp.waypoints[-1].pos, decimal=5
        )

    def test_status_before_start_returns_first_wp(self):
        status = self.fp.status_at_time(-100)
        self.assertIs(status, self.fp.waypoints[0])

    def test_status_returns_waypoint_object(self):
        status = self.fp.status_at_time(5)
        self.assertIsInstance(status, Waypoint)

    def test_status_at_each_wp_time_matches_wp_position(self):
        """At exact WP timestamps position must equal WP position."""
        for wp in self.fp.waypoints:
            status = self.fp.status_at_time(wp.t)
            np.testing.assert_array_almost_equal(
                status.pos, wp.pos, decimal=3,
                err_msg=f"Position mismatch at WP '{wp.label}', t={wp.t}"
            )

    def test_status_at_each_wp_time_matches_wp_velocity(self):
        """At exact WP timestamps velocity must equal WP velocity."""
        for wp in self.fp.waypoints[:-1]:   # last WP has no outgoing segment
            status = self.fp.status_at_time(wp.t + 1e-6)
            diff = np.linalg.norm(status.vel - wp.vel)
            self.assertLess(diff, 0.5,
                msg=f"Velocity mismatch near WP '{wp.label}', t={wp.t}")

    def test_status_position_continuous(self):
        """Position should be continuous: no sudden jumps between adjacent queries."""
        dt = 0.01
        prev = self.fp.status_at_time(self.fp.init_time())
        for t in np.arange(self.fp.init_time() + dt,
                            self.fp.finish_time() - dt, dt):
            curr = self.fp.status_at_time(t)
            jump = np.linalg.norm(curr.pos - prev.pos)
            self.assertLess(jump, 5.0,
                msg=f"Position jump {jump:.2f} m at t={t:.2f}")
            prev = curr


class TestUniformVelocity(unittest.TestCase):
    """set_uniform_velocity: direction, magnitude, last-WP stop."""

    def test_velocity_direction_matches_segment(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=0,  pos=[0, 0, 0])
        fp.set_waypoint(label="B", time=10, pos=[100, 0, 0])
        fp.set_uniform_velocity()
        vel = fp.waypoints[0].vel
        self.assertAlmostEqual(vel[0], 10.0, places=2)
        self.assertAlmostEqual(vel[1], 0.0,  places=2)
        self.assertAlmostEqual(vel[2], 0.0,  places=2)

    def test_last_waypoint_velocity_is_zero(self):
        fp = build_simple_fp()
        fp.set_uniform_velocity()
        last_vel = fp.waypoints[-1].vel
        np.testing.assert_array_almost_equal(last_vel, [0, 0, 0])

    def test_set_speed_adjusts_time(self):
        """Setting a speed of 5 m/s over 100 m should give t = 20 s."""
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=0,   pos=[0,   0, 0])
        fp.set_waypoint(label="B", time=100, pos=[100, 0, 0])
        fp.set_uniform_velocity(wp=0, vel=5)
        self.assertAlmostEqual(fp.waypoints[1].t, 20.0, places=1)

    def test_diagonal_segment_velocity_magnitude(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=0,  pos=[0,  0,  0])
        fp.set_waypoint(label="B", time=10, pos=[60, 80, 0])   # dist = 100 m
        fp.set_uniform_velocity()
        speed = np.linalg.norm(fp.waypoints[0].vel)
        self.assertAlmostEqual(speed, 10.0, places=2)


class TestConnectWaypoints(unittest.TestCase):
    """connect_waypoints: jerk/snap correctness, continuity."""

    def test_connect_produces_nonzero_jerk_when_needed(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=0,  pos=[0,   0, 0], vel=[10, 0, 0])
        fp.set_waypoint(label="B", time=10, pos=[100, 0, 0], vel=[10, 5, 0])
        fp.connect_waypoints()
        self.assertGreater(np.linalg.norm(fp.waypoints[0].jerk), 0)

    def test_straight_uniform_motion_has_zero_jerk(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=0,  pos=[0,   0, 0], vel=[10, 0, 0])
        fp.set_waypoint(label="B", time=10, pos=[100, 0, 0], vel=[10, 0, 0])
        fp.connect_waypoints()
        np.testing.assert_array_almost_equal(fp.waypoints[0].jerk, [0, 0, 0], decimal=8)

    def test_velocity_continuity_at_waypoints(self):
        """The velocity approaching each internal WP equals the WP velocity."""
        fp = build_connected_fp(n_segments=3)
        for wp in fp.waypoints[1:-1]:
            eps = 0.001
            s_before = fp.status_at_time(wp.t - eps)
            diff = np.linalg.norm(s_before.vel - wp.vel)
            self.assertLess(diff, 1.0,
                msg=f"Velocity discontinuity approaching WP '{wp.label}'")


class TestTrace(unittest.TestCase):
    """trace: shape, time axis, last-velocity zero, position match."""

    def setUp(self):
        self.fp = build_connected_fp(n_segments=4)

    def test_trace_has_ten_columns(self):
        tr = self.fp.trace(1.0)
        # columns: t, x, y, z, vx, vy, vz, ax, ay, az
        self.assertEqual(tr.shape[1], 10)

    def test_trace_first_time_equals_init_time(self):
        tr = self.fp.trace(1.0)
        self.assertAlmostEqual(tr[0, 0], self.fp.init_time())

    def test_trace_last_time_near_finish_time(self):
        tr = self.fp.trace(1.0)
        self.assertAlmostEqual(tr[-1, 0], self.fp.finish_time(), delta=1.0)

    def test_trace_last_velocity_is_zero(self):
        tr = self.fp.trace(1.0)
        np.testing.assert_array_almost_equal(tr[-1, 4:7], [0, 0, 0])

    def test_trace_positions_match_waypoints(self):
        tr = self.fp.trace(0.1)
        times = np.round(tr[:, 0], 1)
        for wp in self.fp.waypoints:
            idx = np.where(times == round(wp.t, 1))[0]
            if len(idx) > 0:
                np.testing.assert_array_almost_equal(
                    tr[idx[0], 1:4], wp.pos, decimal=2,
                    err_msg=f"Trace pos mismatch at WP '{wp.label}', t={wp.t}"
                )

    def test_trace_step_determines_resolution(self):
        tr_coarse = self.fp.trace(1.0)
        tr_fine   = self.fp.trace(0.1)
        self.assertGreater(len(tr_fine), len(tr_coarse))


class TestSerialization(unittest.TestCase):
    """to_dict / from_dict roundtrip fidelity."""

    def test_roundtrip_preserves_waypoint_count(self):
        fp = build_connected_fp(n_segments=4)
        fp2 = FlightPlan()
        fp2.from_dict(fp.to_dict())
        self.assertEqual(len(fp2.waypoints), len(fp.waypoints))

    def test_roundtrip_preserves_positions(self):
        fp = build_connected_fp(n_segments=4)
        fp2 = FlightPlan()
        fp2.from_dict(fp.to_dict())
        for w1, w2 in zip(fp.waypoints, fp2.waypoints):
            np.testing.assert_array_almost_equal(w1.pos, w2.pos, decimal=5)

    def test_roundtrip_preserves_velocities(self):
        fp = build_connected_fp(n_segments=4)
        fp2 = FlightPlan()
        fp2.from_dict(fp.to_dict())
        for w1, w2 in zip(fp.waypoints, fp2.waypoints):
            np.testing.assert_array_almost_equal(w1.vel, w2.vel, decimal=5)

    def test_roundtrip_preserves_metadata(self):
        fp = FlightPlan()
        fp.id, fp.priority, fp.radius = 42, 3, 5.0
        fp.set_waypoint(label="X", time=0)
        fp2 = FlightPlan()
        fp2.from_dict(fp.to_dict())
        self.assertEqual(fp2.id, 42)
        self.assertEqual(fp2.priority, 3)
        self.assertAlmostEqual(fp2.radius, 5.0)

    def test_to_lists_consistency(self):
        fp = build_simple_fp()
        times, positions, velocities, acels, jerks, snaps, crackels, headings = fp.to_lists()
        self.assertEqual(len(times),     len(fp.waypoints))
        self.assertEqual(len(positions), len(fp.waypoints))
        for i, wp in enumerate(fp.waypoints):
            self.assertAlmostEqual(times[i], wp.t)
            np.testing.assert_array_equal(positions[i], wp.pos)


class TestCopy(unittest.TestCase):
    """copy: deep independence."""

    def test_copy_has_same_waypoint_count(self):
        fp = build_simple_fp()
        self.assertEqual(len(fp.copy().waypoints), len(fp.waypoints))

    def test_copy_is_deep(self):
        fp = build_simple_fp()
        fp2 = fp.copy()
        fp2.waypoints[0].pos[0] = 9999.0
        self.assertNotAlmostEqual(fp.waypoints[0].pos[0], 9999.0)

    def test_copy_preserves_positions(self):
        fp = build_connected_fp(n_segments=3)
        fp2 = fp.copy()
        for w1, w2 in zip(fp.waypoints, fp2.waypoints):
            np.testing.assert_array_equal(w1.pos, w2.pos)


class TestConflictDetection(unittest.TestCase):
    """compare_to: same plan gives zero distance; offset plan gives large distance."""

    def test_same_plan_zero_distance(self):
        fp = build_connected_fp(n_segments=3)
        distances, _ = fp.compare_to(fp, 1.0)
        for d in distances:
            self.assertAlmostEqual(d, 0.0, places=5)

    def test_offset_plan_large_distance(self):
        fp1 = build_large_fp(5)
        fp2 = fp1.copy()
        for wp in fp2.waypoints:
            wp.pos[1] += 1000.0
        distances, _ = fp1.compare_to(fp2, 1.0)
        self.assertTrue(all(d > 900 for d in distances))

    def test_returns_matching_lengths(self):
        fp1 = build_large_fp(5)
        fp2 = build_large_fp(5)
        distances, times = fp1.compare_to(fp2, 1.0)
        self.assertEqual(len(distances), len(times))

    def test_partial_overlap_in_time(self):
        """Two plans that overlap only partially in time should still produce results."""
        fp1 = build_simple_fp()          # t = 0..20
        fp1.connect_waypoints()
        fp2 = fp1.copy()
        fp2.postpone(5)                  # t = 5..25 — overlap 5..20
        distances, times = fp1.compare_to(fp2, 1.0)
        self.assertGreater(len(distances), 0)


class TestCommandGeneration(unittest.TestCase):
    """get_command and get_isaacsim_command correctness and output types."""

    def setUp(self):
        self.fp  = build_connected_fp(n_segments=3)
        self.pos = np.array([50.0, 5.0, 55.0])
        self.vel = np.array([10.0, 0.0,  0.0])
        self.rot = Rotation.from_euler('z', 0.3)

    def test_get_command_returns_command_instance(self):
        cmd = self.fp.get_command(5.0, self.pos, self.vel, self.rot, None, 0.1)
        self.assertIsInstance(cmd, Command)

    def test_get_command_is_on(self):
        cmd = self.fp.get_command(5.0, self.pos, self.vel, self.rot, None, 0.1)
        self.assertTrue(cmd.on)

    def test_get_command_duration_matches_t_to_solve(self):
        t_to_solve = 0.25
        cmd = self.fp.get_command(5.0, self.pos, self.vel, self.rot, None, t_to_solve)
        self.assertAlmostEqual(cmd.duration, t_to_solve)

    def test_get_command_yaw_rate_in_pi_range(self):
        """rotZ should correspond to a yaw error in [-pi, pi] divided by t_to_solve."""
        t_to_solve = 1.0
        for yaw in [0.0, 1.5, -1.5, 3.1, -3.1]:
            rot = Rotation.from_euler('z', yaw)
            cmd = self.fp.get_command(5.0, self.pos, self.vel, rot, None, t_to_solve)
            self.assertLessEqual(abs(cmd.rotZ), np.pi + 0.01,
                msg=f"rotZ={cmd.rotZ:.3f} out of range for yaw={yaw}")

    def test_get_command_with_explicit_heading(self):
        heading = [0.0, 1.0]   # point north
        cmd = self.fp.get_command(5.0, self.pos, self.vel, self.rot, heading, 0.1)
        self.assertIsInstance(cmd, Command)

    def test_get_isaacsim_command_returns_correct_types(self):
        lin_vel, yaw_rate = self.fp.get_isaacsim_command(
            5.0, self.pos, self.vel, 0.3, None, 0.1
        )
        self.assertEqual(len(lin_vel), 3)
        self.assertIsInstance(float(yaw_rate), float)

    def test_get_isaacsim_command_yaw_error_in_pi_range(self):
        """Yaw error normalisation: result * t_to_solve should be in [-pi, pi]."""
        t_to_solve = 1.0
        for yaw in [0.0, 1.5, -1.5, 4.0, -4.0]:
            _, yaw_rate = self.fp.get_isaacsim_command(
                5.0, self.pos, self.vel, yaw, None, t_to_solve
            )
            yaw_error = yaw_rate * t_to_solve
            self.assertLessEqual(abs(yaw_error), np.pi + 0.01,
                msg=f"yaw_error={yaw_error:.3f} for yaw={yaw}")

    def test_correction_velocity_reduces_position_error(self):
        """A command issued for a displaced UAV should point it back."""
        # UAV is 10 m behind the expected position
        expected = self.fp.status_at_time(5.0)
        behind_pos = expected.pos - np.array([10, 0, 0])
        t_to_solve = 1.0
        lin_vel, _ = self.fp.get_isaacsim_command(
            5.0, behind_pos, self.vel, 0.0, None, t_to_solve
        )
        # The commanded velocity should have a positive X component (catching up)
        self.assertGreater(lin_vel[0], expected.vel[0])


class TestKnownBugs(unittest.TestCase):
    """
    Tests that document known bugs in the codebase.
    These tests are EXPECTED TO FAIL until the bugs are fixed.
    """

    def test_bug_interpolation_sets_crkl_instead_of_crakle(self):
        """
        BUG: waypoint.py Waypoint.interpolation() writes 'wp2.crkl = c2'
        but the correct attribute name is 'wp2.crakle'.
        After any interpolation the .crakle field is never updated.
        """
        wp = Waypoint(
            t=0, pos=[0, 0, 0], vel=[5, 0, 0],
            acel=[0, 0, 0], jerk=[0.1, 0, 0],
            snap=[0.01, 0, 0], crakle=[0.001, 0, 0]
        )
        result = wp.interpolation(5.0)
        # If the bug is present, 'crkl' is set (wrong) and 'crakle' is stale.
        self.assertFalse(
            hasattr(result, 'crkl'),
            "BUG PRESENT: interpolation() sets 'wp2.crkl' (typo) instead of 'wp2.crakle'."
        )

    def test_bug_smooth_waypoint_speed_label_undefined(self):
        """
        BUG: flight_plan.py smooth_waypoint_speed() references 'label' in the
        error message of its first guard clause, but 'label' is not defined in
        that scope — it should reference 'wp'.  This causes a NameError before
        the intended RuntimeError can be raised when i==0 or i==last.
        """
        fp = build_simple_fp()
        fp.set_uniform_velocity()
        # Attempt to smooth the first waypoint (i=0) — should raise RuntimeError
        try:
            fp.smooth_waypoint_speed(0, angVel=0.5)
            self.fail("Expected RuntimeError was not raised")
        except RuntimeError:
            pass   # correct behaviour — bug is fixed
        except NameError as e:
            self.fail(
                f"BUG PRESENT: NameError raised instead of RuntimeError: {e}"
            )


class TestEdgeCases(unittest.TestCase):
    """Boundary conditions and unusual inputs."""

    def test_single_segment_trace(self):
        fp = FlightPlan()
        fp.set_waypoint(label="A", time=0,  pos=[0,   0, 10], vel=[10, 0, 0])
        fp.set_waypoint(label="B", time=10, pos=[100, 0, 10], vel=[0,  0, 0])
        fp.connect_waypoints()
        tr = fp.trace(1.0)
        self.assertGreater(len(tr), 0)

    def test_status_after_finish_extrapolates(self):
        fp = build_simple_fp()
        fp.connect_waypoints()
        # Querying beyond finish should not crash and should return a Waypoint
        status = fp.status_at_time(fp.finish_time() + 100)
        self.assertIsInstance(status, Waypoint)

    def test_postpone_negative_step_advances(self):
        fp = build_simple_fp()
        fp.postpone(-5)
        self.assertAlmostEqual(fp.init_time(), -5)

    def test_compare_to_non_overlapping_plans(self):
        """Two plans that do NOT overlap in time: compare_to should handle gracefully."""
        fp1 = build_simple_fp()    # t = 0..20
        fp1.connect_waypoints()
        fp2 = build_simple_fp()    # will be placed at t = 100..120
        fp2.reschedule_at(100)
        fp2.connect_waypoints()
        # Should not crash; distances list may be empty or very short
        try:
            distances, times = fp1.compare_to(fp2, 1.0)
        except Exception as e:
            self.fail(f"compare_to raised an exception for non-overlapping plans: {e}")

    def test_copy_of_empty_fp(self):
        fp = FlightPlan()
        fp2 = fp.copy()
        self.assertEqual(fp2.waypoints, [])

    def test_trace_empty_plan_not_called(self):
        """trace() on an empty plan would crash; guard against it upstream."""
        fp = FlightPlan()
        self.assertEqual(len(fp.waypoints), 0)


# ===========================================================================
# PERFORMANCE BENCHMARKS
# ===========================================================================

class BenchmarkResult:
    def __init__(self, name: str, time_us: float, note: str = ""):
        self.name    = name
        self.time_us = time_us
        self.note    = note

    def __str__(self):
        return (f"  {self.name:<58} "
                f"{self.time_us:>9.2f} µs   {self.note}")


def _measure(func, iters: int = 1000) -> float:
    """Return average execution time in microseconds."""
    t0 = time.perf_counter()
    for _ in range(iters):
        func()
    return (time.perf_counter() - t0) / iters * 1_000_000


def run_benchmarks(verbose: bool = True) -> list:
    results: list[BenchmarkResult] = []

    def record(name, time_us, note=""):
        r = BenchmarkResult(name, time_us, note)
        if verbose:
            print(r)
        results.append(r)

    # ------------------------------------------------------------------
    # 1. Flight-plan construction — O(n²) because each set_waypoint
    #    performs a linear scan of existing waypoints.
    # ------------------------------------------------------------------
    if verbose:
        print("\n── Build / insertion ─────────────────────────────────────────")
    for n in [10, 50, 100, 500]:
        iters = max(1, 200 // n)
        def _build(n=n):
            fp = FlightPlan()
            for i in range(n):
                fp.set_waypoint(label=f"WP{i}", time=float(i * 10),
                                pos=[float(i * 100), 0, 50], vel=[10, 0, 0])
        t_us = _measure(_build, iters=iters)
        record(f"build_fp (n={n:4d} waypoints)", t_us,
               note="O(n²) — linear scan per insertion")

    # ------------------------------------------------------------------
    # 2. Index lookup — get_target_index_from_time (linear O(n) scan)
    # ------------------------------------------------------------------
    if verbose:
        print("\n── get_target_index_from_time  [O(n) linear scan] ────────────")
    for n in [10, 100, 500, 1000]:
        fp = build_large_fp(n)
        mid_t = fp.waypoints[n // 2].t
        t_us = _measure(lambda fp=fp, t=mid_t: fp.get_target_index_from_time(t),
                        iters=10_000)
        record(f"get_target_index (n={n:4d} wps, midpoint)", t_us,
               note="bisect would give O(log n)")

    # ------------------------------------------------------------------
    # 3. status_at_time — O(n) scan + polynomial interpolation
    # ------------------------------------------------------------------
    if verbose:
        print("\n── status_at_time  [O(n) scan + interpolation] ───────────────")
    for n in [10, 100, 500]:
        fp = build_large_fp(n)
        mid_t = fp.waypoints[n // 2].t + 0.5   # inside a segment
        t_us = _measure(lambda fp=fp, t=mid_t: fp.status_at_time(t),
                        iters=5_000)
        record(f"status_at_time   (n={n:4d} wps, midpoint)", t_us,
               note="bisect would give O(log n)")

    # ------------------------------------------------------------------
    # 4. Trace generation — dominated by number of time-steps
    # ------------------------------------------------------------------
    if verbose:
        print("\n── trace() ───────────────────────────────────────────────────")
    for (n_wp, step) in [(5, 0.1), (10, 0.1), (20, 0.1), (10, 0.01)]:
        fp = build_large_fp(n_wp)
        n_pts = int((fp.finish_time() - fp.init_time()) / step)
        t_us = _measure(lambda fp=fp, s=step: fp.trace(s), iters=50)
        record(f"trace (n_wp={n_wp:3d}, step={step}, ~{n_pts:5d} pts)", t_us)

    # ------------------------------------------------------------------
    # 5. postpone_from — O(n), but very cheap per waypoint
    # ------------------------------------------------------------------
    if verbose:
        print("\n── postpone_from  [O(n)] ─────────────────────────────────────")
    for n in [10, 100, 500, 1000]:
        fp = build_large_fp(n)
        mid_t = fp.waypoints[n // 2].t
        call_count = [0]
        def _postpone(fp=fp, t=mid_t, cc=call_count):
            if cc[0] % 2 == 0:
                fp.postpone_from(t, 1.0)
            else:
                fp.postpone_from(t + 1.0, -1.0)   # undo
            cc[0] += 1
        t_us = _measure(_postpone, iters=2_000)
        record(f"postpone_from    (n={n:4d} wps, midpoint)", t_us)

    # ------------------------------------------------------------------
    # 6. Conflict detection — compare_to
    # ------------------------------------------------------------------
    if verbose:
        print("\n── compare_to (conflict detection) ───────────────────────────")
    for (n_wp, step) in [(5, 1.0), (20, 1.0), (20, 0.1)]:
        fp1 = build_large_fp(n_wp)
        fp2 = build_large_fp(n_wp)
        for wp in fp2.waypoints:
            wp.pos[1] += 50
        n_pts = int((fp1.finish_time() - fp1.init_time()) / step)
        t_us = _measure(lambda fp1=fp1, fp2=fp2, s=step: fp1.compare_to(fp2, s),
                        iters=20)
        record(f"compare_to (n_wp={n_wp:2d}, step={step}, ~{n_pts:4d} pts)", t_us)

    # ------------------------------------------------------------------
    # 7. Navigation command generation
    # ------------------------------------------------------------------
    if verbose:
        print("\n── Command generation ────────────────────────────────────────")
    fp  = build_connected_fp(n_segments=10)
    rot = Rotation.from_euler('z', 0.5)
    pos = np.array([50.0, 10.0, 55.0])
    vel = np.array([10.0,  0.0,  0.0])

    t_us = _measure(lambda: fp.get_command(5.0, pos, vel, rot, None, 0.1),
                    iters=10_000)
    record("get_command (ROS-style)", t_us)

    t_us = _measure(lambda: fp.get_isaacsim_command(5.0, pos, vel, 0.5, None, 0.1),
                    iters=10_000)
    record("get_isaacsim_command", t_us)

    # ------------------------------------------------------------------
    # 8. Serialization
    # ------------------------------------------------------------------
    if verbose:
        print("\n── Serialization (to_dict / from_dict) ───────────────────────")
    for n in [5, 50, 200]:
        fp_s = build_large_fp(n)
        t_us = _measure(lambda fp=fp_s: fp.to_dict(), iters=500)
        record(f"to_dict   (n={n:3d} wps)", t_us)

        d = fp_s.to_dict()
        t_us = _measure(lambda d=d: FlightPlan().from_dict(d), iters=500)
        record(f"from_dict (n={n:3d} wps)", t_us)

    return results


# ===========================================================================
# COMPLEXITY & BUG SUMMARY
# ===========================================================================

SUMMARY = """
╔══════════════════════════════════════════════════════════════════════════════╗
║  ALGORITHMIC COMPLEXITY ANALYSIS                                            ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Method                          │ Current  │ Optimal  │ Notes              ║
║──────────────────────────────────┼──────────┼──────────┼────────────────────║
║  get_target_index_from_time      │  O(n)    │ O(log n) │ use bisect.bisect  ║
║  status_at_time                  │  O(n)    │ O(log n) │ calls above        ║
║  set_waypoint (insertion)        │  O(n)    │ O(n)     │ list.insert — OK   ║
║  postpone_from                   │  O(n)    │ O(n)     │ unavoidable        ║
║  trace(step)                     │  O(T/Δt) │ O(T/Δt)  │ unavoidable        ║
║  compare_to(fp2, step)           │  O(T/Δt) │ O(T/Δt)  │ unavoidable        ║
║  to_dict / from_dict             │  O(n)    │ O(n)     │ OK                 ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  IDENTIFIED BUGS (see TestKnownBugs)                                        ║
║                                                                              ║
║  1. waypoint.py  Waypoint.interpolation()                                   ║
║     Line: "wp2.crkl = c2"                                                   ║
║     Should be: "wp2.crakle = c2"                                            ║
║     Effect: the crakle (5th derivative) field is NEVER updated after any    ║
║     interpolation call; any downstream code reading .crakle gets a stale    ║
║     value (whatever was set at construction time).                          ║
║                                                                              ║
║  2. flight_plan.py  FlightPlan.smooth_waypoint_speed()                      ║
║     Line: raise RuntimeError(f"... (received label: {label})")              ║
║     'label' is not defined in scope; should be 'wp'.                        ║
║     Effect: a NameError is raised instead of the intended RuntimeError      ║
║     when an invalid waypoint index is supplied (i==0 or i==last).           ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""


# ===========================================================================
# ENTRY POINT
# ===========================================================================

if __name__ == "__main__":
    print("=" * 80)
    print("  FlightPlan — Correctness & Performance Test Suite")
    print("=" * 80)

    # ── Unit tests ─────────────────────────────────────────────────────────
    print("\n[1/3] Running correctness tests …\n")
    loader = unittest.TestLoader()
    suite  = unittest.TestSuite()
    for cls in [
        TestFlightPlanBasic,
        TestWaypointInsertion,
        TestWaypointRemoval,
        TestIndexLookup,
        TestTimeManagement,
        TestStatusAtTime,
        TestUniformVelocity,
        TestConnectWaypoints,
        TestTrace,
        TestSerialization,
        TestCopy,
        TestConflictDetection,
        TestCommandGeneration,
        TestEdgeCases,
        TestKnownBugs,
    ]:
        suite.addTests(loader.loadTestsFromTestCase(cls))

    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(suite)

    # ── Benchmarks ──────────────────────────────────────────────────────────
    print("\n[2/3] Running performance benchmarks …")
    bench_results = run_benchmarks(verbose=True)

    # ── Summary ─────────────────────────────────────────────────────────────
    print("\n[3/3] Analysis summary")
    print(SUMMARY)

    # ── Final verdict ────────────────────────────────────────────────────────
    n_tests   = result.testsRun
    n_fail    = len(result.failures)
    n_error   = len(result.errors)
    n_known   = 2   # TestKnownBugs has 2 expected-failure tests
    n_ok      = n_tests - n_fail - n_error

    print(f"Tests run : {n_tests}")
    print(f"  Passed  : {n_ok}")
    print(f"  Failed  : {n_fail}  "
          f"(up to {n_known} are KNOWN BUGS documented in TestKnownBugs)")
    print(f"  Errors  : {n_error}")

    sys.exit(0 if result.wasSuccessful() else 1)
