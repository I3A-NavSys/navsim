

# This module defines the SweptBox class for continuous collision detection (CCD) in 3D space.
# The SweptBox represents the volume swept by the UAV moving from one position to another over a time interval.
# This box is an axis-aligned bounding box (AABB) that encompasses the start and end positions of the UAV, expanded by a safety radius.

# We need to try with Oriented Bounding Boxes (OBBs) in the future, to see if it reduces the number of false positives in the collision detection, at the cost of more complex collision checks.

# We need also to see if we should worry about curves in the trayectory.
# With the current implementation, we are assuming that the UAV moves in a straight line between the start and end positions. 
# If the UAV follows a curved path, we might need to sample points along the curve and create multiple SweptBoxes 
# to better approximate the swept volume.

# Or even we can use the 7D formula to sample in a more exact way.

import numpy as np



class SweptBox:
    def __init__(self, p_min, p_max, t_start, t_end):
        self.min = np.array(p_min)  # Inferior corner [x, y, z]
        self.max = np.array(p_max)  # Superior corner [x, y, z]
        self.t_range = (t_start, t_end)

    def collides_with(self, other):
        """
        Continuous Collision Detection (CCD) using AABB.
        If the boxes do not overlap on any axis, there is no collision.
        """
        # Quick check in 3D (Broad-phase)
        # (self.max < other.min) checks if this box is completely to the left/below/behind the other box on any axis.
        # (self.min > other.max) checks if this box is completely to the right/above/in front of the other box on any axis.
        # np.any compares the 3 axes (x, y, z) and returns True if any of the conditions are met, indicating no collision.
        return not (np.any(self.max < other.min) or np.any(self.min > other.max))