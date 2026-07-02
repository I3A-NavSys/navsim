# flight_plan  (Rust port)

Direct Rust translation of the optimised `FlightPlan` / `Waypoint` from
`uspace/flight_plan/flight_plan_new.py` and `waypoint_new.py`.

The port keeps the public API, doc-comments and overall structure as
close to the Python original as possible — see the file-by-file
translation table below.

## Project layout

```text
flight_plan (Python)            flight_plan (Rust)
├── command.py                  ├── src/command.rs
├── waypoint_new.py             ├── src/waypoint.rs
└── flight_plan_new.py          └── src/flight_plan.rs
```

## Building

The crate has **no external dependencies** — only `std` is used. The
build needs a stable Rust toolchain (`rustc` + `cargo`) and a C linker
(`cc` on PATH, or `CC_x86_64_unknown_linux_gnu` set).

```bash
cd uspace/flight_plan/rust
cargo build --release
```

This produces the benchmark binary at
`target/release/benchmark`.

## Translation status

| Python                                               | Rust                                     | Status                       |
|------------------------------------------------------|------------------------------------------|------------------------------|
| `Waypoint.__init__`                                  | `Waypoint::new`                          | translated                   |
| `Waypoint.copy`                                      | `Waypoint::copy`                         | translated                   |
| `Waypoint.stop`                                      | `Waypoint::stop`                         | translated                   |
| `Waypoint.postpone`                                  | `Waypoint::postpone`                     | translated                   |
| `Waypoint.time_to`                                   | `Waypoint::time_to`                      | translated                   |
| `Waypoint.distance_to`                               | `Waypoint::distance_to`                  | translated                   |
| `Waypoint.direction_to`                              | `Waypoint::direction_to`                 | translated                   |
| `Waypoint.course_to`                                 | `Waypoint::course_to`                    | translated                   |
| `Waypoint.angle_with`                                | `Waypoint::angle_with`                   | translated                   |
| `Waypoint.set_uniform_velocity`                      | `Waypoint::set_uniform_velocity`         | translated                   |
| `Waypoint.connect_to`                                | `Waypoint::connect_to`                   | translated (Cramer's rule)   |
| `Waypoint.interpolate`                               | `Waypoint::interpolate`                  | translated                   |
| `Command.__init__` / `set` / `off` / `hover`         | `Command::new` / `set` / `off` / `hover` | translated                   |
| `FlightPlan.__init__`                                | `FlightPlan::new`                        | translated                   |
| `FlightPlan.add_waypoint`                            | `FlightPlan::add_waypoint`               | translated                   |
| `FlightPlan.remove_waypoint`                         | `FlightPlan::remove_waypoint`            | translated                   |
| `FlightPlan.get_idx_by_id`                           | `FlightPlan::get_idx_by_id`              | translated                   |
| `FlightPlan.get_running_waypoint*`                   | `FlightPlan::get_running_waypoint*`      | translated (binary search)   |
| `FlightPlan.get_target_waypoint*`                    | `FlightPlan::get_target_waypoint*`       | translated (binary search)   |
| `FlightPlan.start_time` / `finish_time`              | `FlightPlan::start_time` / `finish_time` | translated                   |
| `FlightPlan.postpone_from`                           | `FlightPlan::postpone_from`              | translated                   |
| `FlightPlan.postpone` / `reschedule_at`              | `FlightPlan::postpone` / `reschedule_at` | translated                   |
| `FlightPlan.connect_waypoints`                       | `FlightPlan::connect_waypoints`          | translated                   |
| `FlightPlan.status_at_time`                          | `FlightPlan::status_at_time`             | translated                   |
| `FlightPlan.trace`                                   | `FlightPlan::trace`                      | translated (vectorised)      |
| `FlightPlan.copy`                                    | `FlightPlan::copy`                       | translated                   |
| `FlightPlan.waypoints_to_arrays`                     | _inline in `trace`_                      | inlined (private helper)     |
| `FlightPlan.to_dict` / `from_dict`                   | —                                        | not ported (no I/O needed)   |
| `FlightPlan.rotate_vector_by_quaternion`             | —                                        | not ported (UI helper)       |
| `FlightPlan.make_plan_feasible`                      | —                                        | not ported                   |
| `FlightPlan.smooth_waypoint_speed` / `_duration`     | —                                        | not ported                   |
| `FlightPlan.get_command`                             | —                                        | not ported                   |
| `FlightPlan.compare_to` / `print_comparison`         | —                                        | not ported                   |
| `FlightPlan.plot_position` / `velocity` / ...        | —                                        | not ported (matplotlib)      |
| `FlightPlan.add_UAV_track_*`                         | —                                        | not ported                   |
| `ToggleUAVtracking`                                  | —                                        | not ported (matplotlib UI)   |

The non-ported methods are intentionally left out of the Rust port —
the goal is an apples-to-apples benchmark of the kinematic core, not a
re-implementation of the matplotlib UI or the dict-based
serialisation.

## Running the comparison

The Python harness in
`benchmarks/run_comparison.py` runs both implementations with the same
operations and the same input sizes, and prints a side-by-side table.

```bash
cd uspace/flight_plan/rust
cargo build --release
python3 benchmarks/run_comparison.py
```

Sample output (with `n` up to 500):

```text
BENCHMARK                  N          PYTHON         RUST     SPEEDUP
create+connect            500       12.72 ms     60.10 us     211.6x
connect_waypoints         500        3.55 ms     15.28 us     232.6x
status_at_time x2k        500       25.80 ms     40.35 us     639.3x
trace                     500       11.92 ms    550.54 us      21.7x
postpone                  500       15.02 us      9.84 us       1.5x

Geometric-mean speedup of Rust over Python: 62.4x
```

## Design notes

* `Vec3` and `Vec2` are plain `[f64; N]` arrays. They are
  stack-allocated, copy-cheap, and they behave like the original
  `numpy.array([...])` shape `(3,)` objects for all the operations
  used by `FlightPlan`.

* `time_waypoints: Vec<f64>` is a parallel sorted array of waypoint
  times. The Python version uses `SortedList` from the
  `sortedcontainers` package; in Rust we just use a `Vec` + manual
  `bisect_left` / `bisect_right` (no extra crate).

* `connect_to` solves the 3×3 `A·X = B` system with **Cramer's rule**
  evaluated once per axis, instead of pulling in `nalgebra`. The
  matrix `A` depends only on `t12`, so the determinant is computed
  once and re-used.

* `trace` writes its 19-column output into a single flat `Vec<f64>` of
  length `n_samples * 19`. There is no per-sample allocation and no
  intermediate numpy object.
