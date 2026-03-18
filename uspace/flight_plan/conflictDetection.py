

# This module defines the SweptBox class for continuous collision detection (CCD) in 3D space.
# The SweptBox represents the volume swept by the UAV moving from one position to another over a time interval.
# This box is an axis-aligned bounding box (AABB) that encompasses the start and end positions of the UAV, expanded by a safety radius.

# We implemented Oriented Bounding Boxes (OBBs) to reduce the number of false positives in collision detection.
# OBBs are rotated to align with the movement direction, providing better accuracy at the cost of more complex collision checks.

# Note: The current implementation assumes that the UAV moves in a straight line between the start and end positions.
# If the UAV follows a curved path, we might need to sample points along the curve and create multiple SweptBoxes 
# to better approximate the swept volume.

# In the future, we could use the 7D formula for more exact sampling along curved trajectories.

import numpy as np



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
        
        POTENTIAL BUGS (currently NOT validated):
        1. No validation that t_start <= t_end
           → Silent bug if t_range = (10, 5)
        2. No validation that axes form orthogonal basis
           → SAT assumes orthogonal axes, non-orthogonal will give wrong results
        3. No validation that half_extents > 0
           → Negative or zero extents will cause projection errors
        4. Floating-point tolerance in SAT comparisons
           → Edge cases like max1 == min2 may fail due to rounding
        """
        self.center = np.array(center, dtype=float) # Center of the box [x, y, z]
        self.axes = np.array(axes, dtype=float)  # 3x3 matrix
        self.half_extents = np.array(half_extents, dtype=float) # Half-lengths along each axis [hx, hy, hz]
        self.t_range = (t_start, t_end) # Time interval [t_start, t_end]
    
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
        
        POTENTIAL BUGS:
        
        1. NEAR-ZERO AXIS NORMALIZATION:
           If axis norm ≈ 0, division by (norm + 1e-10) gives unstable results
           → Projection values become very large
           → Can cause false negatives in SAT collision checks
           → Recommend: if norm < epsilon, skip this axis
        
        2. ASSUMPTION: ORTHOGONAL AXES:
           This method assumes self.axes are orthonormal.
           If axes are skewed or non-unit length:
           → Projections will be incorrect
           → No validation occurs
        
        3. FLOATING-POINT ACCUMULATION:
           sum(abs(...) * ...) can accumulate rounding errors
           → Slightly wrong proj_half values
           → Cascades through SAT comparisons
        """
        axis = np.array(axis)
        axis_norm = axis / (np.linalg.norm(axis) + 1e-10)
        
        # Project center
        proj_center = np.dot(self.center, axis_norm)
        
        # Project half-extents
        proj_half = sum(abs(np.dot(self.axes[i], axis_norm)) * self.half_extents[i] 
                       for i in range(3))
        
        return proj_center - proj_half, proj_center + proj_half
    
    def collides_with(self, other):
        """
        OBB-OBB collision detection using Separating Axis Theorem (SAT).
        Returns True if boxes collide, False otherwise.
        
        POTENTIAL BUGS & EDGE CASES:
        
        1. TEMPORAL EDGE CASE - Boxes touching in time:
           If self.t_range = [0, 5] and other.t_range = [5, 10]:
           → Condition: 5 < 5? NO → Proceeds to spatial check
           → Current behavior: Treats touching intervals as overlapping
           → May cause false positives depending on intention
        
        2. FLOATING-POINT TOLERANCE IN SAT:
           Comparisons like 'max1 < min2' with floats can fail:
           Example: 4.9999999999 not < 5.0000000001 due to rounding
           → Can produce false negatives (misses real collisions)
           → Recommend using epsilon tolerance: `max1 < min2 - EPSILON`
        
        3. ASSUMES ORTHOGONAL AXES:
           SAT projects along axes assuming they're orthonormal.
           If axes are NOT orthogonal (due to previous bugs),
           → SAT gives incorrect results silently
           → No validation before entering loop
        
        4. ZERO-EXTENT AXES:
           If any half_extents[i] == 0 or very small,
           → proj_half approaches 0
           → Box degenerates to line or point
           → SAT may still work but semantics unclear
        """
        # First, check if the time intervals overlap. If they don't, there's no collision.
        # if self.t_range[1] < other.t_range[0] or \
        #     other.t_range[1] < self.t_range[0]:
        #     return False  # No temporal overlap → no collision

        # WE ALREADY DO THIS IN THE CONFLICT DETECTION LOOP IN PLAYGROUND.PY TO AVOID CHECKING EVERY SINGLE BOX AGAINST EVERY OTHER BOX
        
        # Test axes from this OBB
        for i in range(3):
            min1, max1 = self.project_on_axis(self.axes[i])
            min2, max2 = other.project_on_axis(self.axes[i])
            if max1 < min2 or max2 < min1:
                return False  # Separated on this axis
        
        # Test axes from other OBB
        for i in range(3):
            min1, max1 = self.project_on_axis(other.axes[i])
            min2, max2 = other.project_on_axis(other.axes[i])
            if max1 < min2 or max2 < min1:
                return False  # Separated on this axis
        
        # Test cross products (9 more axes)
        for i in range(3):
            for j in range(3):
                cross_axis = np.cross(self.axes[i], other.axes[j])
                if np.linalg.norm(cross_axis) > 1e-10:  # Avoid zero vectors
                    min1, max1 = self.project_on_axis(cross_axis)
                    min2, max2 = other.project_on_axis(cross_axis)
                    if max1 < min2 or max2 < min1:
                        return False  # Separated on this axis
        
        return True  # No separating axis found - collision detected
