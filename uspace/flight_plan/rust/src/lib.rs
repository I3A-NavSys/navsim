//! # flight_plan  (Rust port)
//!
//! Rust port of the optimised FlightPlan / Waypoint from
//! `uspace/flight_plan/flight_plan_new.py` and `waypoint_new.py`.
//!
//! The structure mirrors the original Python package one-to-one:
//!
//! ```text
//! flight_plan (Python)               flight_plan (Rust)
//! ├── command.py                     ├── src/command.rs
//! ├── waypoint_new.py                ├── src/waypoint.rs
//! └── flight_plan_new.py             └── src/flight_plan.rs
//! ```
//!
//! The goal is to keep the public API, doc-comments and overall structure as
//! close to the Python original as possible. Methods that are not part of the
//! numerical hot-path (matplotlib-based plotting, dict-based serialisation,
//! smoothing heuristics) are intentionally left out — the focus is on the
//! kinematic core that the Python-vs-Rust benchmark exercises.

pub mod command;
pub mod waypoint;
pub mod flight_plan;

pub use command::Command;
pub use waypoint::Waypoint;
pub use flight_plan::{FlightPlan, Trace};
