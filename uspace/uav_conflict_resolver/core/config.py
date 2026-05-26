# =============================================================================
# GLOBAL CONFIGURATION: UAV 4D Conflict Resolver
# =============================================================================
# All tunable parameters for the resolution pipeline are centralized here.
# Changing these values affects every phase of the resolution cascade.
# =============================================================================

# -----------------------------------------------------------------------------
# NUMERICAL TOLERANCES
# These were originally defined in detection/conflictDetection.py.
# They are centralised here so that all modules share a single source of truth.
#
# EPSILON_ABSOLUTE:   Detects pure numerical instabilities (values near zero).
#                     Used for near-zero norm/axis checks throughout SAT.
#
# EPSILON_ORTHOGONAL: Validates orthonormality of rotation matrices and unit
#                     vectors. Required by the SAT algorithm.
#
# EPSILON_RELATIVE:   Accounts for cascading floating-point errors that scale
#                     with operand magnitude. For UAVs operating in the 1–100 m
#                     range, 1 mm (1e-3) tolerance is acceptable.
# -----------------------------------------------------------------------------
EPSILON_ABSOLUTE:   float = 1e-9   # Near-zero detection (numerical noise floor)
EPSILON_ORTHOGONAL: float = 1e-6   # Orthonormality / unit-vector validation
EPSILON_RELATIVE:   float = 1e-3   # Scaled tolerance for collision comparisons [m]

# Derived tolerance used inline inside collides_with() [see conflictDetection.py].
# Pre-calculated for the typical ~100 m UAV operating scale.
SAT_EPSILON: float = max(EPSILON_ABSOLUTE, EPSILON_RELATIVE * 100)

# General-purpose near-zero check for position/velocity norms across all modules.
# Reuse EPSILON_ABSOLUTE for this — defined as a readable alias.
NORM_ZERO_THRESHOLD: float = EPSILON_ABSOLUTE   # alias: np.linalg.norm(x) < this → zero

# Small time gap inserted between adjacent waypoints to avoid overwriting existing
# entries when using set_waypoint() or postpone_from() with an offset.
WAYPOINT_TIME_EPSILON: float = 0.001   # [s]

# Numerical stabiliser used as a denominator guard in division operations.
# Prevents division-by-zero in ramp alpha calculations.
DIV_EPSILON: float = 1e-12

# -----------------------------------------------------------------------------
# WCET ANCHOR (Temporal Safety)
# The computation always starts at t_anchor = max(t_conflict, t_start) + ANCHOR_DELTA.
# ANCHOR_DELTA = WCET + SAFETY_MARGIN ensures the drone has enough time to receive
# the new plan before the maneuver is physically needed.
# NEVER plan a maneuver at t=0 or before the UAV's scheduled departure (t_start).
# -----------------------------------------------------------------------------
WCET_SECONDS:           float = 3.0   # Worst-Case Execution Time of the resolver [s]
SAFETY_MARGIN_SECONDS:  float = 3.0   # Additional buffer on top of WCET [s]
ANCHOR_DELTA:           float = WCET_SECONDS + SAFETY_MARGIN_SECONDS  # = 3.0 s

# -----------------------------------------------------------------------------
# STRATEGY 1: Kinematic Bounding (time-shift of nearest waypoint)
#
# Finds the waypoint closest to the conflict instant and shifts its timestamp
# by ±delta seconds.  Both signs are tried for each delta, so no direction
# pre-selection is needed — connect_waypoints() + the R-Tree decide.
#
# S1_TIME_SHIFTS: candidate deltas to try, in order [s].
# With velocity-adaptive scaling, these base deltas are multiplied by
# (reference_speed / actual_speed), providing broader coverage across UAV speeds.
# -----------------------------------------------------------------------------
S1_TIME_SHIFTS: list = [1.0, 2.0, 5.0, 10.0]   # Candidate time shifts [s]

# -----------------------------------------------------------------------------
# PHYSICAL UAV LIMITS
# Used by the kinematic feasibility check in connect_waypoints().
#
# UAV_MAX_SPEED: The drone's absolute top speed. A segment requiring a higher
#                average speed than this is physically impossible.
#
# UAV_MAX_ACCEL: The drone's peak linear acceleration. Used to compute the
#                maximum reachable distance from a given initial speed in a
#                given time window (Guard 3 in check_kinematic_feasibility).
#                Note: this bound is optimistic — it assumes the drone
#                accelerates at full thrust for the entire segment duration,
#                which is only achievable if no deceleration is needed at the
#                end. Hence the check is a necessary condition, not sufficient.
# -----------------------------------------------------------------------------
UAV_MAX_SPEED: float = 20.0   # Maximum drone speed                  [m/s]
UAV_MAX_ACCEL: float = 5.0    # Maximum drone linear acceleration     [m/s²]
# Fallback cruise speed used when the flight plan reports no/zero speed
# (prevents divisions by zero and provides a reasonable heuristic).
CRUISE_SPEED_FALLBACK: float = 10.0  # [m/s]

# -----------------------------------------------------------------------------
# STRATEGY 2 / FALLBACK 1: SAT MTV Generation
# After Strategy 1 fails, the resolver computes one MTV per SAT axis by re-running
# the 15-axis SAT loop (3 plebeian OBB axes + 3 VIP OBB axes + 9 cross products).
# This gives up to 15 independent displacement candidates, one per axis.
# Each MTV is multiplied by SAFETY_MTV_SCALE to add a small clearance margin
# beyond the bare minimum separation distance.
# The candidates are split into:
#   - Horizontal_MTVs (XY-dominant): tried by Strategy 2 (Path Stretch)
#   - Vertical_MTVs  (Z-dominant):   tried by Fallback 1 (Vertical MTVs)
# Both lists are sorted ascending by magnitude (minimum disruption first).
#
# DETOUR_APEX_TIME_FRACTION:   Fraction of the inter-waypoint time budget used
#                              to reach the spatial detour apex (Legacy Fallback).
# DETOUR_RETURN_TIME_FRACTION: Fraction at which the return waypoint is placed.
#                              Must satisfy: APEX < RETURN < 1.0
# -----------------------------------------------------------------------------
SAFETY_MTV_SCALE:            float = 1.05  # 5% margin beyond bare minimum [dimensionless]
DETOUR_APEX_TIME_FRACTION:   float = 0.40  # 40% to detour apex
DETOUR_RETURN_TIME_FRACTION: float = 0.75  # 75% to return point

# -----------------------------------------------------------------------------
# FALLBACK 2: Time-Shift (Hovering)
# The plebeian UAV brakes to 0 m/s at t_anchor and hovers in increments of
# HOVER_TIME_STEP until the R-Tree confirms the path is clear.
# HOVER_MAX_TIMEOUT is the hard cutoff to prevent infinite loops.
# -----------------------------------------------------------------------------
HOVER_MAX_TIMEOUT: float = 60.0   # Maximum hovering duration before DEADLOCK [s]
HOVER_TIME_STEP:   float = 0.5    # Time increment per hovering iteration [s]

# -----------------------------------------------------------------------------
# R-TREE / OBB SAMPLING
# Interval used when (re)generating swept OBB boxes for R-Tree registration.
# Smaller = more boxes = more precise detection but higher memory usage.
# -----------------------------------------------------------------------------
OBB_INTERVAL: float = 0.5   # Sampling interval for OBB box generation [s]

# Default physical radius used when generating swept OBBs and AABBs
# Represents the UAV's safety radius (half-width/half-height of OBB lateral axes) [m]
UAV_RADIUS: float = 3.5

# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------
# RIGID SHIFT DETOUR (STRATEGY 2 / FALLBACK 1)
#
# Parameters for the new Rigid Shift implementation of Strategy 2.
#
# RIGID_SHIFT_MTV_SCALE:
#   Internal multiplier applied to the raw SAT MTV before displacing the
#   detour point.  The SAT MTV is the *minimum* geometric separation; the
#   quintic polynomial can reduce effective clearance by curving the path back
#   toward the obstacle.  A factor of 2.0 adds a large safety margin on top
#   of the bare minimum, reducing the risk of residual overlap after smoothing.
#
# MTV_SCALE_TIME_BUFFER_FACTOR:
#   Proportional temporal buffer for MTV scaling iterations.
#   The buffer is calculated dynamically as:
#     buffer = conflict_duration * MTV_SCALE_TIME_BUFFER_FACTOR
#   When MTV is scaled up (1.0, 1.25, 1.5, 2.0), the anchor and return waypoint 
#   times are adjusted by (scale - 1.0) * buffer, giving earlier start and later 
#   end for larger MTVs. This prevents secondary conflicts at maneuver boundaries
#   on long conflicts by scaling the temporal margins with conflict length.
# 
# EXAMPLE: If conflict lasts 30s and factor=0.15, buffer = 4.5s per scale unit.
# For scale=1.5: t_detour_start delayed by 2.25s, t_detour_end advanced by 2.25s.
# ---
RIGID_SHIFT_MTV_SCALE:          float = 2.0   # Internal MTV scale in Rigid Shift detour [dimensionless]
MTV_SCALE_TIME_BUFFER_FACTOR:   float = 0.15  # Temporal buffer = conflict_duration * this factor

# -----------------------------------------------------------------------------
# SHADOW R-TREE (Forward Progress Margin)
# Used by _validate_temporary_swap to ensure any remaining conflicts after a
# maneuver occur comfortably in the future, preventing immediate Temporal
# Domino Effects.
# This value must stay strictly greater than the largest S1_TIME_SHIFTS entry
# so S1 does not accept a secondary conflict that is still too close to the
# conflict being solved.
# -----------------------------------------------------------------------------
FORWARD_PROGRESS_MARGIN:        float = max(S1_TIME_SHIFTS) + 2.0   # Time buffer for subsequent conflicts [s]

# Constants for skipping intermediate waypoints during detour creation
MIN_DETOUR_DURATION:     float = 5.0    # Minimum duration allocated for detour [s]
POST_CONFLICT_BUFFER:    float = 1.0    # Time buffer after conflict end to select target waypoint [s]
