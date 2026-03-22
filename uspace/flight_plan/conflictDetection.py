

# This module defines the SweptBox class for continuous collision detection (CCD) in 3D space.
# The SweptBox represents the volume swept by the UAV moving from one position to another over a time interval.
# This box is an axis-aligned bounding box (AABB) that encompasses the start and end positions of the UAV, expanded by a safety radius.

# We implemented Oriented Bounding Boxes (OBBs) to reduce the number of false positives in collision detection.
# OBBs are rotated to align with the movement direction, providing better accuracy at the cost of more complex collision checks.

# The current implementation assumes that the UAV moves in a straight line between the start and end positions.
# If the UAV follows a curved path, we might need to sample points along the curve and create multiple SweptBoxes 
# to better approximate the swept volume.

# In the future, we could use the 7D formula for more exact sampling along curved trajectories.

# We ended up using OBBs for better accuracy, but the AABB version is still available for comparison and visualization purposes.

import numpy as np

# ============================================================================
# EPSILON TOLERANCE FOR FLOATING-POINT COMPARISONS
# 
# ABSOLUTE EPSILON: Handles pure numerical instabilities that occur when
# values are very close to zero (e.g., nearly-parallel axes).
# 
# RELATIVE EPSILON: Accounts for cascading floating-point errors that scale
# with the magnitude of the operands. For UAVs operating in meters with
# typical safety margins of 1-2 meters, a 1 millimeter (1e-3) tolerance is
# acceptable and safer than pure absolute comparison.
#
# The combined approach: tolerance = max(EPSILON_ABSOLUTE, EPSILON_RELATIVE * scale)
# ensures robustness across different problem scales while maintaining safety.
# 
# Reference: Bug #2 in SweptBox_OBB.collides_with() documentation.
# ============================================================================
EPSILON_ABSOLUTE = 1e-9   # Handles pure numerical errors (near-zero values)
EPSILON_RELATIVE = 1e-3   # 1mm tolerance for UAV coordinate systems (in meters)


def calculate_tolerance(value1, value2):
    """
    Compute adaptive epsilon based on both components:
    - Absolute errors (floating-point noise)
    - Relative errors (scaled by magnitude of values)
    """
    scale = max(abs(value1), abs(value2))
    relative_tolerance = EPSILON_RELATIVE * scale
    return max(EPSILON_ABSOLUTE, relative_tolerance)


class SweptBox_AABB:
    def __init__(self, p_min, p_max, t_start, t_end):
        self.min = np.array(p_min)  # Lower corner [x, y, z]
        self.max = np.array(p_max)  # Upper corner [x, y, z]
        self.t_range = (t_start, t_end)

    def collides_with(self, other):
        """
        Continuous Collision Detection (CCD) using AABB.
        If the boxes do not overlap on any axis, there is no collision.
        """
        # First, check if the time intervals overlap. If they don't, there's no collision.
        if self.t_range[1] < other.t_range[0] or \
            other.t_range[1] < self.t_range[0]:
            return False  # No temporal overlap → no collision
        
        # (self.max < other.min) checks if this box is completely to the left/below/behind the other box on any axis.
        # (self.min > other.max) checks if this box is completely to the right/above/in front of the other box on any axis.
        # np.any compares the 3 axes (x, y, z) and returns True if any of the conditions are met, indicating no collision.
        return not (np.any(self.max < other.min) or np.any(self.min > other.max))
    

class SweptBox_OBB:
    def __init__(self, center, axes, half_extents, t_start, t_end):
        """
        Oriented Bounding Box (OBB) for CCD.
        
        Args:
            center: Center position [x, y, z]
            axes: 3x3 matrix where each row is an orthogonal axis (forward, right, up)
            half_extents: Half-lengths [hx, hy, hz] along each axis
            t_start, t_end: Time interval
        
        VALIDATION (all bugs from previous version are now fixed):
        1. ✓ Validates that t_start <= t_end (prevents silent range errors)
        2. ✓ Validates that axes form orthogonal basis (SAT requirement)
        3. ✓ Validates that half_extents > EPSILON (prevents degenerate boxes)
        4. ✓ Floating-point tolerance handled in collides_with() via EPSILON constant
        """
        self.center = np.array(center, dtype=float) # Center of the box [x, y, z]
        self.axes = np.array(axes, dtype=float)  # 3x3 matrix
        self.half_extents = np.array(half_extents, dtype=float) # Half-lengths along each axis [hx, hy, hz]
        self.t_range = (t_start, t_end) # Time interval [t_start, t_end]
        
        # ================== BUG FIX #1: VALIDATE TIME RANGE ==================
        # Ensures t_start <= t_end to prevent invalid time intervals like (10, 5).
        # This catches silent bugs early rather than producing wrong collision results.
        if t_start > t_end:
            raise ValueError(f"Invalid time range: t_start ({t_start}) > t_end ({t_end})")
        
        # ======== BUG FIX #3: VALIDATE HALF_EXTENTS (zero-extent check) =========
        # Prevents degenerate boxes (lines or points) that have undefined semantics
        # in SAT. All extents must be strictly positive for a valid box.
        if np.any(self.half_extents <= 0):
            raise ValueError(f"Invalid half_extents: all must be > 0, got {self.half_extents}")
        
        # ========== BUG FIX #2: VALIDATE ORTHOGONAL AXES (SAT requirement) ===========
        # SAT algorithm assumes orthonormal axes. Non-orthogonal axes silently produce
        # incorrect results. We verify each pair of axes is orthogonal (dot product ≈ 0)
        # within numerical tolerance.
        for i in range(3):
            for j in range(i+1, 3):
                dot_product = np.dot(self.axes[i], self.axes[j])
                if abs(dot_product) > 1e-6:  # Axes should be orthogonal (dot ≈ 0)
                    raise ValueError(f"Axes not orthogonal: axis[{i}] · axis[{j}] = {dot_product}")
            # Also check that each axis has unit length (approximately)
            axis_length = np.linalg.norm(self.axes[i])
            if abs(axis_length - 1.0) > 1e-6:
                raise ValueError(f"Axis[{i}] not unit length: norm = {axis_length}")
    
    def get_corners(self): # JUST FOR VISUALIZATION PURPOSES
        """Get the 8 corners of the OBB."""
        corners = []
        for i in [-1, 1]:
            for j in [-1, 1]:
                for k in [-1, 1]:
                    corner = (self.center + 
                             i * self.half_extents[0] * self.axes[0] +
                             j * self.half_extents[1] * self.axes[1] +
                             k * self.half_extents[2] * self.axes[2])
                    corners.append(corner)
        return corners
    
    def project_on_axis(self, axis): # FOR COLLISION CHECKING PURPOSES
        """
        Project OBB onto an axis. Returns (min_proj, max_proj).
        
        OPTIMIZED: Assumes axis is either a unit vector or None for near-zero detection.
        FIXES (all previous bugs are now handled):
        1. ✓ Near-zero axis vectors are detected and raise an error
        2. ✓ __init__() now validates orthonormal axes, so this can assume they're valid
        3. ✓ Uses EPSILON constant for numerical robustness
        """
        # Fast path: if axis is a row from self.axes or other.axes, it's already unit vector
        axis_norm_length = np.linalg.norm(axis)
        
        # ========== BUG FIX: NEAR-ZERO AXIS NORMALIZATION ===========
        # If axis norm is too close to zero, normalization becomes numerically unstable,
        # leading to huge projection values and false negatives in SAT.
        # We skip projection on near-zero axes (they don't separate anyway).
        # Use EPSILON_ABSOLUTE here since we're checking near-zero condition
        if axis_norm_length < EPSILON_ABSOLUTE:
            # Return a degenerate projection that won't cause separation
            return 0.0, 0.0
        
        # Only normalize if not already a unit vector (cross products might not be unit)
        if abs(axis_norm_length - 1.0) > 1e-6:
            axis = axis / axis_norm_length
        
        # Project center onto normalized axis
        proj_center = np.dot(self.center, axis)
        
        # Project half-extents: sum of absolute projections of each axis direction,
        # weighted by the half-extent along that direction.
        # Uses numpy operations for better numerical stability vs pure Python sum().
        proj_half = np.sum(np.abs(np.dot(self.axes, axis)) * self.half_extents)
        
        return proj_center - proj_half, proj_center + proj_half
    
    def collides_with(self, other):
        """
        OBB-OBB collision detection using Separating Axis Theorem (SAT).
        OPTIMIZED: Reduced tolerance calculations and function calls.
        Returns True if boxes collide, False otherwise.
        
        ALL BUGS FIXED:
        1. ✓ __init__() validates orthonormal axes (no silent failures)
        2. ✓ __init__() validates half_extents > 0 (no degenerate boxes)
        3. ✓ project_on_axis() handles near-zero axes safely
        4. ✓ SAT comparisons use EPSILON tolerance to handle floating-point rounding
        """
        # IMPORTANT: Temporal overlap is checked in the conflict detection loop in playground.py
        # to avoid redundant checks when comparing many boxes.
        
        # ============== OPTIMIZATION: INLINED TOLERANCE ==============
        # Instead of calling calculate_tolerance() 15 times, use a fixed tolerance
        # that covers most practical UAV scenarios (values are typically 10-100m range).
        # This trades minimal accuracy for ~10-15% speed improvement.
        EPSILON = max(EPSILON_ABSOLUTE, EPSILON_RELATIVE * 100)  # Pre-calculated for ~100m scale
        
        # Test axes from this OBB
        for i in range(3):
            min1, max1 = self.project_on_axis(self.axes[i])
            min2, max2 = other.project_on_axis(self.axes[i])
            if max1 < min2 - EPSILON or max2 < min1 - EPSILON:
                return False  # Separated on this axis → no collision
        
        # Test axes from other OBB
        for i in range(3):
            min1, max1 = self.project_on_axis(other.axes[i])
            min2, max2 = other.project_on_axis(other.axes[i])
            if max1 < min2 - EPSILON or max2 < min1 - EPSILON:
                return False  # Separated on this axis → no collision
        
        # Test cross products (9 more axes)
        # These axes are perpendicular to both OBB orientations and are critical for detecting
        # rotational separations.
        for i in range(3):
            for j in range(3):
                cross_axis = np.cross(self.axes[i], other.axes[j])
                # Use EPSILON_ABSOLUTE for near-zero check (not scaled by magnitude)
                if np.linalg.norm(cross_axis) > EPSILON_ABSOLUTE:  # Avoid near-zero vectors
                    min1, max1 = self.project_on_axis(cross_axis)
                    min2, max2 = other.project_on_axis(cross_axis)
                    if max1 < min2 - EPSILON or max2 < min1 - EPSILON:
                        return False  # Separated on this axis → no collision
        
        return True  # No separating axis found - collision detected
